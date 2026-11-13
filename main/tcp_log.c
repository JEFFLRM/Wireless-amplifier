/**
 * tcp_log.c - TCP 远程日志 (ESP-IDF v5.x / lwIP)
 *
 * 架构:
 *   ┌──────────────┐   vprintf 回调    ┌───────────────┐   send()    ┌──────────┐
 *   │ ESP_LOGx()   │ ──────────────→ │ s_log_cb()     │ ─────────→ │ TCP Client│
 *   └──────────────┘                  └──────┬────────┘              └──────────┘
 *                                            │ 同时写 USB UART
 *                                            ▼
 *                                     原始 vprintf (UART)
 *
 *   accept_task: socket() → bind() → listen() → accept() 循环, 新连接踢掉旧 client
 *
 * 线程安全: s_client 访问用 s_mux 互斥, send() 用非阻塞模式避免卡住日志调用方.
 * 可靠性: best-effort; client 断开时 send() 返回错误 → 标记 INVALID → 等待下次 accept.
 */

#include "tcp_log.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>  /* va_list / va_copy / va_end */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"

#define TAG        "tcp_log"
#define DEF_PORT   3333
#define BACKLOG    1
#define RECV_BUF   512   /* 随便收 client 发来的东西, 保持连接活跃 */

static int           s_server  = -1;
static int           s_client  = -1;
static SemaphoreHandle_t s_mux = NULL;
static TaskHandle_t  s_accept_task_h = NULL;
static bool          s_inited = false;

/* 保存原始 vprintf (USB UART), 在我们的回调里继续调用, 保证串口日志不丢 */
static vprintf_like_t s_orig_vprintf = NULL;

/**
 * @brief 所有 ESP_LOG 的总出口: 同时写给 TCP client 和 USB UART
 *
 * 注意: 这个回调可能被多个任务并发调用, 也可能在中断禁用区 (虽然少见).
 *       所以只用 mutex (不是 recursive), 且 send 非阻塞, 绝不 vTaskDelay.
 */
static int s_log_cb(const char *fmt, va_list ap)
{
    /* 1. 先写 USB UART (原始路径, 不丢日志) —— 用 va_copy 保护 ap
     *    注意: va_list 被 vsnprintf/vprintf 消费后不能再用, 必须 va_copy */
    va_list ap_uart;
    va_copy(ap_uart, ap);
    int uart_len = s_orig_vprintf ? s_orig_vprintf(fmt, ap_uart) : 0;
    va_end(ap_uart);

    /* 2. 再写 TCP client (best-effort, 绝不阻塞)
     *    必须在 accept_task 里打印 client 状态; 这里绝不调 ESP_LOGx —— 会递归进本回调 */
    if (s_mux && xSemaphoreTake(s_mux, 0) == pdTRUE) {
        if (s_client >= 0) {
            va_list ap_tcp;
            va_copy(ap_tcp, ap);
            char buf[256];
            int n = vsnprintf(buf, sizeof(buf), fmt, ap_tcp);
            va_end(ap_tcp);
            if (n > 0) {
                /* MSG_NOSIGNAL (lwip 已定义为 0x20, 注释说 Unimplemented, 但传了无害) */
                int sent = lwip_send(s_client, buf, n, MSG_NOSIGNAL);
                if (sent < 0) {
                    lwip_close(s_client);
                    s_client = -1;
                    /* 不调 ESP_LOG —— 会递归进本回调, 丢失日志 */
                }
            }
        }
        xSemaphoreGive(s_mux);
    }
    return uart_len; /* 返回 UART 输出长度, esp_log 框架需要 */
}

/**
 * @brief accept 循环任务: 监听端口, 接受新 client, 保持单一客户端模式
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
    if (lwip_listen(s_server, BACKLOG) < 0) {
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

        /* 新 client 连上: 踢掉旧的 */
        if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
            if (s_client >= 0) {
                ESP_LOGW(TAG, "new client connected, kicking old one");
                lwip_close(s_client);
            }
            s_client = c;

            /* 设非阻塞: send 绝不卡住日志回调; recv 用于 keep-alive 检测 */
            int flags = lwip_fcntl(c, F_GETFL, 0);
            lwip_fcntl(c, F_SETFL, flags | O_NONBLOCK);

            char ip[16];
            inet_ntoa_r(cli.sin_addr, ip, sizeof(ip));
            ESP_LOGI(TAG, "client %s:%u connected, now streaming logs", ip, ntohs(cli.sin_port));
            xSemaphoreGive(s_mux);
        }

        /* keep-alive: 尝试收 client 发的任意数据 (比如 telnet 的 IAC 控制序列),
         * 如果 recv 返回 0 或错误, 说明 client 主动关了或异常断开 */
        char dummy[RECV_BUF];
        while (1) {
            int r = lwip_recv(c, dummy, sizeof(dummy), 0);
            if (r == 0) {
                ESP_LOGI(TAG, "client closed connection");
                break;
            } else if (r < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    vTaskDelay(pdMS_TO_TICKS(200)); /* 周期性探测 */
                    continue;
                }
                ESP_LOGW(TAG, "client recv errno=%d, closing", errno);
                break;
            }
            /* r > 0: client 发了东西, 丢掉继续等 */
        }

        /* client 已断开 */
        if (xSemaphoreTake(s_mux, portMAX_DELAY) == pdTRUE) {
            if (s_client == c) {
                lwip_close(s_client);
                s_client = -1;
            }
            xSemaphoreGive(s_mux);
        }
        /* 回到 accept 继续等下一个 client */
    }
}

esp_err_t tcp_log_init(uint16_t port)
{
    if (s_inited) return ESP_OK;

    if (port == 0) port = DEF_PORT;

    s_mux = xSemaphoreCreateMutex();
    if (!s_mux) return ESP_ERR_NO_MEM;

    s_orig_vprintf = esp_log_set_vprintf(s_log_cb); /* 替换日志回调, 保存原始 (UART) */
    s_inited = true;

    BaseType_t ok = xTaskCreatePinnedToCore(s_accept_task, "tcp_log", 4096,
                                            (void *)(uintptr_t)port, 2,
                                            &s_accept_task_h, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        esp_log_set_vprintf(s_orig_vprintf); /* 恢复原始 */
        vSemaphoreDelete(s_mux);
        s_mux = NULL;
        s_inited = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
