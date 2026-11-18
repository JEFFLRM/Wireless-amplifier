/**
 * tcp_log.c - TCP 远程日志 (ESP-IDF v5.x / lwIP)
 *
 * 架构 (v2: 日志回调不再直接碰 socket):
 *   ┌──────────────┐  vprintf 回调   ┌──────────────┐ 环形缓冲 ┌──────────────┐
 *   │ ESP_LOGx()   │ ─────────────→ │ s_log_cb()   │ ───────→ │ s_send_task  │ → TCP Client
 *   └──────────────┘                 │  + UART 直写  │          └──────────────┘
 *                                    └──────────────┘
 *
 * 旧版直接 vprintf 里 lwip_send/lwip_close, 有两个致命问题:
 *   1) lwip_send 返回 EAGAIN 时直接 lwip_close, 而 accept 任务正在对同一 fd 调
 *      lwip_recv, 形成 use-after-free → 硬故障/死机 (串口也无输出)
 *   2) 在任何任务(含 lwIP 线程)的日志路径里调用 lwIP API, 可能重入 lwIP 核心锁
 *
 * 现在: 日志回调只做 UART + 环形缓冲写入, 完全不碰 socket;
 *       客户端 fd 只由 s_send_task 一个任务持有 (accept/send/recv/close 全在其中).
 */

#include "tcp_log.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>  /* va_list / va_copy / va_end */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "esp_system.h"

#define TAG         "tcp_log"
#define DEF_PORT    3333
#define LOG_SB_SIZE 8192  /* 日志环形缓冲 (满则丢最旧, best-effort) */
#define SEND_CHUNK  1024  /* 每次从缓冲取出送 socket 的块大小 */
#define RECV_BUF    128   /* 用于探测 client 是否关闭 */

static int                s_server = -1;
static int                s_client = -1;  /* 仅 s_send_task 访问, 无需加锁 */
static TaskHandle_t       s_task_h = NULL;
static bool               s_inited = false;
static StreamBufferHandle_t s_log_sb = NULL; /* 日志回调 → 发送任务 的单向数据通道 */

/* 保存原始 vprintf (USB UART), 在我们的回调里继续调用, 保证串口日志不丢 */
static vprintf_like_t s_orig_vprintf = NULL;

/**
 * @brief 所有 ESP_LOG 的总出口: 先写 USB UART, 再入环形缓冲
 *
 * 关键约束: 这里绝不调用任何 lwIP API (不 send / 不 close / 不 take 互斥锁),
 *           因为本回调可能在任何任务甚至 lwIP 线程的上下文中被调用.
 */
static int s_log_cb(const char *fmt, va_list ap)
{
    /* 1. 先写 USB UART (原始路径, 不丢日志) —— 用 va_copy 保护 ap
     *    注意: va_list 被 vsnprintf/vprintf 消费后不能再用, 必须 va_copy */
    va_list ap_uart;
    va_copy(ap_uart, ap);
    int uart_len = s_orig_vprintf ? s_orig_vprintf(fmt, ap_uart) : 0;
    va_end(ap_uart);

    /* 2. 格式化后塞进环形缓冲, 由 s_send_task 负责发 TCP (不阻塞日志调用方) */
    if (s_log_sb != NULL) {
        va_list ap_tcp;
        va_copy(ap_tcp, ap);
        char buf[256];
        int n = vsnprintf(buf, sizeof(buf), fmt, ap_tcp);
        va_end(ap_tcp);
        if (n > 0) {
            size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
            /* 超时 0 = 缓冲满就丢这行, 绝不阻塞 */
            xStreamBufferSend(s_log_sb, buf, len, 0);
        }
    }
    return uart_len; /* 返回 UART 输出长度, esp_log 框架需要 */
}

/**
 * @brief 服务单个 client 直到断开: 取日志 → 发 → 探测对端关闭
 * @return 无 (返回后调用方负责 close 并回到 accept)
 */
static void s_serve_client(int c)
{
    char rx[RECV_BUF];
    char tx[SEND_CHUNK];

    while (1) {
        /* 取一批日志 (最多等 100ms, 没数据就顺便探测对端) */
        size_t n = xStreamBufferReceive(s_log_sb, tx, sizeof(tx), pdMS_TO_TICKS(100));
        if (n > 0) {
            size_t off = 0;      /* 已发送字节数 */
            int retry = 0;       /* EAGAIN 重试计数 */
            while (off < n) {
                int s = lwip_send(c, tx + off, n - off, 0);
                if (s > 0) {
                    off += (size_t)s;
                    retry = 0;
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    /* 窗口满: 短暂退让后重试, 超过 5s 仍发不出去视为客户端失联 */
                    if (++retry > 50) return;
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                return; /* 真错误: 连接已坏 */
            }
            continue; /* 还有日志就继续发, 不打断探测节奏 */
        }

        /* 100ms 内没有日志: 探测 client 是否主动关闭/异常断开 */
        int r = lwip_recv(c, rx, sizeof(rx), 0);
        if (r == 0) return; /* FIN: 对端正常关闭 */
        if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return; /* 异常 */
        /* r > 0: client 发了数据(如 telnet 控制符), 丢弃继续 */
    }
}

/**
 * @brief accept 循环任务: 监听端口, 串行服务单一 client
 *
 * 客户端 fd 只在本任务中出现, 避免与日志回调并发操作同一 fd 导致的崩溃.
 */
static void s_accept_task(void *arg)
{
    uint16_t port = (uint16_t)(uintptr_t)arg;

    s_server = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (s_server < 0) {
        ESP_LOGE(TAG, "socket() failed errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int yes = 1;
    lwip_setsockopt(s_server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(port),
        .sin_addr   = { .s_addr = htonl(INADDR_ANY) },
    };
    if (lwip_bind(s_server, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind port %u failed errno=%d", port, errno);
        lwip_close(s_server);
        s_server = -1;
        vTaskDelete(NULL);
        return;
    }
    if (lwip_listen(s_server, 1) < 0) {
        ESP_LOGE(TAG, "listen failed errno=%d", errno);
        lwip_close(s_server);
        s_server = -1;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCP log server listening on 0.0.0.0:%u (telnet/nc to receive ESP_LOG)", port);

    while (1) {
        struct sockaddr_in cli;
        socklen_t clen = sizeof(cli);
        int c = lwip_accept(s_server, (struct sockaddr *)&cli, &clen);
        if (c < 0) {
            ESP_LOGW(TAG, "accept errno=%d, retry", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* 非阻塞: send 遇窗口满可立即返回, 交给 s_serve_client 退让重试 */
        int flags = lwip_fcntl(c, F_GETFL, 0);
        lwip_fcntl(c, F_SETFL, flags | O_NONBLOCK);

        s_client = c; /* 本任务独占该 fd */
        xStreamBufferReset(s_log_sb); /* 丢掉上一个 client 遗留的旧日志 */

        char ip[16];
        inet_ntoa_r(cli.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "client %s:%u connected, now streaming logs", ip, ntohs(cli.sin_port));

        s_serve_client(c); /* 阻塞服务直到断开 (send 失败 / 对端关闭 / 对端失联) */

        lwip_close(c);
        s_client = -1;
        ESP_LOGI(TAG, "client disconnected, waiting for new connection");
    }
}

esp_err_t tcp_log_init(uint16_t port)
{
    if (s_inited) return ESP_OK;

    if (port == 0) port = DEF_PORT;

    s_log_sb = xStreamBufferCreate(LOG_SB_SIZE, 1);
    if (!s_log_sb) return ESP_ERR_NO_MEM;

    s_orig_vprintf = esp_log_set_vprintf(s_log_cb); /* 替换日志回调, 保存原始 (UART) */
    s_inited = true;

    BaseType_t ok = xTaskCreatePinnedToCore(s_accept_task, "tcp_log", 4096,
                                            (void *)(uintptr_t)port, 2,
                                            &s_task_h, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        esp_log_set_vprintf(s_orig_vprintf); /* 恢复原始 */
        vStreamBufferDelete(s_log_sb);
        s_log_sb = NULL;
        s_inited = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
