/**
 * receiver.c - 7.1CH 无线音频接收端 (ESP32-C5, 5GHz WiFi STA)
 * 单声道单播 RTP 接收 + XOR FEC 丢包恢复 + 发现广播 (声道+IP)
 *
 * 协议 v1.3 (PC 发送端按此实现, 1ms 流与批量流并存):
 *   - RTP 头: v_p_x_cc=0x80, pt=96(1ms数据)/97(1ms FEC)/100(批量数据)/101(批量 FEC),
 *     seq/ts/ssrc 网络序
 *   - 1ms 数据包: 每包 48 帧 @48kHz x 1 声道 x 32bit(4字节小端) = 192B 载荷,
 *     总长 204B (RTP头12+载荷192) 小于 MTU 不分片
 *   - 批量数据包 (v1.3): 载荷 = B 个连续 1ms 载荷拼接 (B=2~7, 最大 1344B / 336 帧),
 *     包 ts 为首帧墙钟, 每包 ts 增量 = 48 x B; 接收端按 1ms 拆帧入环,
 *     因 due 调度逐帧判定, 量化误差无感
 *   - FEC: 1ms 流每 10 包异或生成 1 个校验包 (N=10); 批量流每 5 包 1 个 (N=5,
 *     恢复窗口 = 5 x B ms, 保证恢复出的包仍在抖动预算内);
 *     发送端 FEC 包 seq = 本组首包 seq - 1 (即 FEC 包 seq=F 保护 [F+1, F+N], uint16 回绕)
 *   - SSRC = 0x7C41; 声道序号 slot0..7 = L/R/C/LFE/SL/SR/SBL/SBR
 *   - 发现广播: 拿到 IP 后向本网段广播地址的 UDP 6000 端口发送 21 字节,
 *     周期自适应: 未收到音频流时每 5s 一次, 收到音频流后每 20s 一次:
 *     RTP 头(pt=98, seq 递增, ts=开机毫秒) + id(1B, 配网下发的设备编号 0~255)
 *     + slot_bit(1B, 声道按 bit: 1<<slot) + flags(1B: bit0=tx_enable 1=正常发送/0=停止,
 *     bit1=caps 1=支持批量流 PT100/101) + 本机IPv4(4B 网络序)
 *     + CRC16-CCITT-FALSE 包尾(对前 19 字节计算, 小端 2B), 供发送端自动发现
 *     配网下发新配置时: 工作模式 BLE/WiFi 共存, 直接在当前 (旧) WiFi 上连发 3 个
 *     tx_enable=0 的停止通告 (无需重连), 再重启用新配置连 WiFi (重启后 tx_enable 恒为 1)
 *
 * 工作链路: WiFi STA 5GHz 入网 -> UDP 收本机声道 RTP/FEC (socket 绑定固定 5004 端口,
 *           纯单播) -> FEC 缓存恢复丢包 -> 环形缓冲+水位抗抖动
 *           -> I2S 输出 48kHz/32bit 到 ACM8815 数字功放 (见 acm8815.c), 喇叭直接接输出级
 *
 * 工作模式 (NVS 配置, 复用发送端 BLE 配网框架):
 *   无配置            -> BLE 配网模式 (关 WiFi 开 BLE)
 *   mode = "sta"      -> STA 连接 5GHz 路由器 (接收单播音频流, 推荐)
 *   mode = "ap"       -> STA 连接 5GHz SoftAP (直连发送端热点场景, 保留兼容)
 *   长按 BOOT 键 3 秒清空配置重启进入 BLE 配网。
 *
 * 输出:
 *   output_mode="single" -> I2S 标准单声道 (i2s_std.h), 只输出 slot_index 声道
 *   output_mode="full"   -> I2S TDM 8 slot (i2s_tdm.h), 输出全部 8 声道
 *   帧缓冲: 环形缓冲 + 水位控制抗抖动; 丢包恢复失败时该帧输出静音。
 *   注意: ACM8815 为单声道 I2S 数字功放 (喇叭直推), 适用 single 模式 (mono 数据左右槽相同,
 *         功放任取一路均为正确声道); full(TDM 8ch) 模式需 TDM 功放, ACM8815 无法接收 TDM。
 *
 * 注意: ESP32-C5 为单核, 所有任务固定到 Core 0。
 * 依赖: ESP-IDF v5.4+ (ESP32-C5, 5GHz WiFi, NimBLE)
 */
#include <string.h> /* 字符串操作库 (memcpy/memset/strncpy 等) */
#include <stdlib.h> /* 标准库 (assert/malloc 等) */
#include <stdio.h> /* 标准 IO (snprintf 等) */
#include <math.h> /* 数学库 (sinf, 测试音生成) */
#include "freertos/FreeRTOS.h" /* FreeRTOS 内核头文件 */
#include "freertos/task.h" /* 任务 API (xTaskCreate/vTaskDelay) */
#include "freertos/semphr.h" /* 信号量 (互斥锁, ring 并发保护) */
#include "esp_log.h" /* ESP 日志 (ESP_LOGI/LOGW/LOGE) */
#include "esp_system.h" /* ESP 系统 API (esp_restart 等) */
#include "esp_heap_caps.h" /* 堆能力分配 (heap_caps_malloc, MALLOC_CAP_SPIRAM) */
#include "esp_timer.h" /* 高精度定时器 (统计间隔计时) */
#include "esp_netif_sntp.h" /* esp_netif_sntp_init/deinit + esp_sntp_config_t: 对时发端 */
#include "esp_sntp.h"        /* sntp_get_sync_status / sntp_set_time_sync_notification_cb: 对时状态查询与回调 */
#include <sys/time.h>   /* gettimeofday: 绝对墙钟 (SNTP 同步后即 NTP 时间) */
#include <time.h>       /* localtime_r/strftime: 授时日志格式化本地时间 */
#include <arpa/inet.h>  /* inet_ntoa_r: 把 from.sin_addr 转点分十进制字符串, 用于 SNTP server 地址 */
#include "esp_wifi.h" /* WiFi API (esp_wifi_init/set_band 等) */
#include "esp_netif.h" /* 网络接口 (netif) */
#include "esp_event.h" /* 事件循环 */
#include "nvs_flash.h" /* NVS flash 存储 (配置持久化) */
#include "lwip/sockets.h" /* LWIP socket (UDP 单播收包) */
#include "lwip/inet.h" /* LWIP 网络字节序与地址转换 (ntohs/htonl/inet_addr) */
#include "driver/gpio.h" /* GPIO 驱动 (BOOT 键输入) */
#include "driver/i2s_std.h" /* I2S 标准模式 (single 单声道输出) */
#include "driver/i2s_tdm.h" /* I2S TDM 模式 (full 8 声道输出) */

#include "ble_provision.h" /* 本工程配网模块 (prov_config_t 及接口) */
#include "acm8815.h" /* 本工程功放驱动 (ACM8815 I2C 配置) */
#include "ws2812.h" /* 本工程状态灯驱动 (WS2812B RMT) */
#include "ota_update.h" /* 本工程 WiFi OTA 升级 (HTTP 上传固件) */
#include "tcp_log.h" /* TCP 远程日志 (telnet 3333 接收 ESP_LOG) */
#include "esp_ota_ops.h" /* OTA 分区查询 (打印当前运行分区, 便于确认升级是否切换成功) */

#define TAG "receiver" /* 日志标签 */

/* ---------- 协议常量 (与发送端严格一致) ---------- */
#define SAMPLE_RATE    48000 /* 目标采样率 48kHz; 发送端需同步为 48000,
                                  MCLK=384fs=18.432MHz, BCLK=64fs=3.072MHz (32bit 槽) */
#define CH_NUM         8                  /* 系统总声道数 (7.1), 仅用于文档/声道名表; 单播流每包只含 1 个声道 */
#define BIT_DEPTH      32 /* 位深 32bit */
#define FRAMES_PKT     48                 /* 每包帧数 (1ms @48kHz) */
#define BYTES_SAMPLE   (BIT_DEPTH / 8)    /* 每样本字节数 = 4 */
#define PAYLOAD_LEN    (FRAMES_PKT * BYTES_SAMPLE)  /* 单声道载荷 = 48帧 x 4B = 192B (每包只含本机 1 个声道) */
#define FEC_N          10                 /* 1ms 流: 每 FEC_N 个数据包生成 1 个 XOR 校验包 */
#define RTP_PT         96                 /* 1ms 数据包 PT */
#define RTP_PT_FEC     97                 /* 1ms FEC 包 PT */
/* ---- v1.3 批量流 (1~7ms/包) ---- */
#define RTP_PT_BATCH     100              /* 批量数据包 PT */
#define RTP_PT_BATCH_FEC 101              /* 批量 FEC 包 PT */
#define BATCH_MAX_B      7                /* 单包最大 7ms (7 x 48 = 336 帧, 载荷 1344B, 含 RTP 头 1356B < 1400 不分片) */
#define FEC_N_BATCH      5                /* 批量流 FEC 组大小: 恢复窗口 5 x B ms */
#define BATCH_MAX_PAYLOAD (PAYLOAD_LEN * BATCH_MAX_B) /* 批量载荷上限 = 1344B (336 帧) */
#define SSRC           0x7C41 /* 本系统流的同步源标识, 用于过滤其他流 */

/* ---------- 调试开关 ---------- */
#define TEST_TONE_EN   0 /* 1=绕过网络直接 1kHz 正弦 → 切开 "网络PCM vs I2S链路" */

#define DEF_AUDIO_PORT 5004 /* 音频单播接收端口 (固定, 不做组播; 发送端按发现包通告的 IP 单播到此端口) */

/* ---------- 硬件参数 ---------- */
#define WIFI_5G_CHANNEL    0 /* STA 信道: 0=全 5GHz 频段自动扫描; 149~165=锁定 5.8GHz 信道 */
#define BOOT_KEY_GPIO      GPIO_NUM_28      /* C5 BOOT 键 (strapping pin, 低电平按下) */
#define BOOT_KEY_HOLD_MS   3000 /* BOOT 键长按判定阈值 (毫秒) */
#define I2S_GPIO_MCLK      GPIO_NUM_2      /* MCLK 主时钟 (ACM8815 内部 PLL 以 BCLK 为参考,
                                              不需要 MCLK, 此脚输出仅兼容旧布线, 功放侧悬空) */
#define I2S_GPIO_BCLK      GPIO_NUM_3      /* DAC BCLK  */
#define I2S_GPIO_WS        GPIO_NUM_4      /* DAC LRCK  */
#define I2S_GPIO_DOUT      GPIO_NUM_5      /* DAC DIN   */

/* ---------- FEC 缓存 / 环形缓冲参数 ---------- */
#define FEC_CACHE_SZ   58                 /* 包缓存槽数, 必须覆盖 水位+FEC组+突发 (48+10+余量);
                                            20 槽时 drain 落后 >20 包会被 hash 同槽覆盖, 造成额外假丢包.
                                            每槽 ~1.4KB (批量包 1344B), 58 槽约 79KB, 整体放 PSRAM */
#define FEC_GROUP_REC  8                  /* 记录最近 FEC 组状态 (80ms 窗口) */
#define RING_PKTS      160                /* 环形缓冲容量: 160 包 = 160ms (给丢包/突发留足余量, 正常不靠它控延迟) */
#define TARGET_WMARK   48                 /* 起播水位: 攒够 48 包(48ms)再起播, 这就是抖动缓冲垫;
                                           * 之后收发都是 1 包/ms, 水位自然稳定在 48 附近 */
#define RING_HIGH_PKTS 100                /* 积压上限: 突发/静音填充导致水位上涨超过该值 */
#define RING_TRIM_PKTS 70                 /* 超限时丢最旧包回到该水位 (延迟回到 ~70+8ms) */

/* ---------- 定时播出参数 ---------- */
/* 播放时钟: late_ms = (时钟 - ts) + DMA - PLAYOUT_LEAD
 * 只用于判定"落后太多就丢该包", 以及时基跳变后重锚定.
 * 关键: i2s 绝不主动等待 —— 写入 DMA 满时自然阻塞 ≈1ms, 本身就是 1 包/ms 的节拍器;
 * 之前在 i2s 里 vTaskDelay 等待会让消费端停摆 → ring 塞满 → drain 无法推进 →
 * 缓存被新包覆盖 → 每秒几十次 "gap too large" 跳包 (音频完全错乱). */
#define PLAYOUT_LEAD_MS   100             /* 播放延迟 (ms): 包从 ts 到出声的目标延迟 (须 >= 水位+DMA, 否则全部判"迟到")
                                           * 本地锚定模式下它只是偏移量, 留大些以免误判; 墙钟模式即真实播出延迟 */
#define I2S_DMA_DEPTH_MS  8               /* DMA 深度 (4×96帧 = 384帧 = 8ms), 纠偏时补偿 */
#define LATE_DROP_MS      25              /* 比播出时刻晚 >25ms 才丢该包 (水位正常时不丢) */
#define DRAIN_FILL_MAX    16              /* 单次排水最多补多少个静音 (保证缺包后几 ms 内追上最新包) */
#define GAP_JUMP_PKTS     32              /* 落后最新包超过该值就跳时间轴, 不再补几十 ms 静音 */
#define TS_SANE_MS        2000            /* ts 与本地墙钟偏差超过 2s 视为发送端 ts 非墙钟, 回退本地锚定 */
#define SYNC_WAIT_MS      10000           /* 起播前最长等待对时 (方案 A §F1 超时兜底): 超时降级为按 seq 播放 */
#define NTP_SYNC_INTERVAL_MS 20000        /* NTP 刷新周期 20s (RFC4330 下限 15s); 压小多箱间晶振漂移 */

/* RTP 头 (与 sender.c 完全一致: 12 字节, packed) */
typedef struct __attribute__((packed)) { /* RTP 固定头结构 (12 字节, 强制 1 字节对齐, 与 sender.c 完全一致) */
    uint8_t  v_p_x_cc;   /* 0x80 (RTP v2) */
    uint8_t  pt; /* 载荷类型: 96=数据包 / 97=FEC 校验包 */
    uint16_t seq; /* RTP 序号 (网络序, 使用时转主机序) */
    uint32_t ts; /* 时间戳 (本工程未使用, 保留 RTP 头完整) */
    uint32_t ssrc; /* 同步源标识 */
} rtp_header_t; /* 结构体定义结束 */
#define RTP_HDR_LEN  sizeof(rtp_header_t) /* RTP 头长度 = 12 字节 */

/* ---------- 全局状态 (单核/任务间共享) ---------- */
static prov_config_t     s_cfg; /* 全局配网配置 (NVS 加载, 任务间共享) */
static i2s_chan_handle_t s_tx      = NULL; /* I2S 发送通道句柄 */
static bool              s_full;            /* true=full 8ch TDM / false=single mono */
static uint32_t          s_pkt_bytes;       /* 写入 ring 的单包字节: 单声道流恒为 192B (48帧 x 4B) */
static uint16_t          s_slot_index;      /* single 模式提取的声道 0~7 */

/* 任务句柄 (OTA 升级时临时降优先级用) */
static TaskHandle_t s_h_i2s  = NULL;
static TaskHandle_t s_h_net  = NULL;
static TaskHandle_t s_h_disc = NULL;
static TaskHandle_t s_h_stat = NULL;

/* 统计 */
static struct { /* 统计结构体: 网络/I2S 运行计数 */
    uint32_t rx_data;      /* 收到的数据包 */
    uint32_t rx_fec;       /* 收到的 FEC 包 */
    uint32_t recovered;    /* FEC 恢复成功的包 */
    uint32_t missed;       /* 静音占位帧(包)数 */
    uint32_t dropped;      /* ring 满丢弃的新包 */
    uint32_t underrun;     /* ring 空输出静音次数 */
    uint32_t seq_dup;      /* 重复 seq 丢弃 */
    uint32_t bad_ssrc;     /* SSRC 不符丢弃 */
    uint32_t rx_short;     /* 短于 RTP 头的包 (诊断: 小包/UDP 可达性测试) */
    uint32_t rx_badlen;    /* 长度 != 头+载荷 丢弃 (诊断: IP 分片重组失败) */
    uint32_t fec_bad;      /* FEC 组缺 >=2 包无法恢复的次数 (仅汇总统计, 不逐组打印) */
    uint32_t rx_batch;     /* 收到的批量数据包 (pt=100) */
    uint32_t rx_batch_fec; /* 收到的批量 FEC 包 (pt=101) */
    uint32_t missed_batch; /* 批量流补静音帧数 (ms, 与 1ms 流分开统计) */
    uint32_t late_drop;    /* 因"时间戳落后"被丢弃的帧数 (没声音的直接原因之一) */
    uint32_t late_jump;    /* 时基跳变重锚次数 (late < -500ms: 发端重启/换曲/ts 基准突变) */
    uint32_t late_fix;     /* 严重滞后强制重建时基次数 (late > 250ms: 时基长期不一致) */
} s_stats; /* 全局统计实例 */

/* 最近一次收到音频数据包的时刻 (微秒); 0=从未收到。发现任务据此动态调整广播周期 */
static volatile int64_t s_last_rx_us = 0;

/* ============ FEC 恢复缓存 ============ */
typedef struct { /* FEC 缓存槽: 按 seq 散列缓存最近收到的数据包 */
    uint16_t seq; /* 缓存包的 RTP 序号 */
    bool     valid; /* 槽位有效标志 */
    uint16_t len; /* 载荷字节数: 192=1ms 包 / 192 的倍数 (最大 1344)=批量包 */
    uint32_t ts; /* sender 的 RTP ts (wall_ms × 48), 用于绝对墙钟定时 */
    uint8_t  data[BATCH_MAX_PAYLOAD]; /* 完整载荷 (1ms 192B / 批量最多 1344B) */
} pkt_cache_t; /* 缓存槽类型定义结束 */

static pkt_cache_t *s_cache;             /* FEC 包缓存 (FEC_CACHE_SZ 槽, app_main 中分配到 PSRAM) */

typedef struct { /* FEC 组状态记录: 标记某组是否已收到/可恢复 */
    uint16_t fec_seq;      /* FEC 包 seq */
    uint8_t  state;        /* 0=未用, 1=已收且组内可恢复/冗余, 2=已收但组内缺>1 无法恢复 */
    uint8_t  window;       /* 组大小: 10=1ms 流 / 5=批量流 */
    bool     valid; /* 记录有效标志 */
} fec_group_state_t; /* 组状态类型定义结束 */

static fec_group_state_t s_fec_groups[FEC_GROUP_REC]; /* 最近 FEC 组状态环形记录 (8 项, 80ms 窗口) */

static uint16_t s_last_played = 0; /* 已播放到的最大包 seq */
static bool     s_has_last    = false; /* 是否已确定播放起点 */
static uint32_t s_last_ts     = 0; /* 最后入队包的时间戳 (静音包据此合成单调递增 ts, 保证时间轴不被静音打断) */
static uint8_t  s_batch_b     = 1; /* 当前流每包毫秒数: 1=1ms 流 / 2~7=批量流 (按最近收到的数据包学习) */

/* seq 回绕安全比较 (窗口远小于 32768) */
static int16_t seq_diff(uint16_t a, uint16_t b) { return (int16_t)(a - b); } /* 回绕安全差值: uint16 相减转 int16 判正负 */
static bool seq_gt(uint16_t a, uint16_t b) { return seq_diff(a, b) > 0; } /* 回绕安全比较: a > b */
static bool seq_lt(uint16_t a, uint16_t b) { return seq_diff(a, b) < 0; } /* 回绕安全比较: a < b */

static void cache_put(uint16_t seq, uint32_t ts, const uint8_t *payload, uint16_t len) /* 写入缓存槽: 按 seq 散列定位 */
{ /* 函数体开始 */
    int idx = seq % FEC_CACHE_SZ; /* 散列槽号: seq 对槽数取模 */
    s_cache[idx].seq   = seq; /* 记录序号 */
    s_cache[idx].len   = len; /* 记录载荷长度 (192 / 批量 192 的倍数, 最大 1344) */
    s_cache[idx].ts    = ts;  /* 记录 sender 的 RTP ts (wall_ms × 48) */
    s_cache[idx].valid = true; /* 标记槽有效 */
    memcpy(s_cache[idx].data, payload, len); /* 拷贝完整载荷到缓存槽 */
} /* 函数结束 */

static pkt_cache_t *cache_get(uint16_t seq) /* 查询缓存包: 槽有效且序号一致才命中 */
{ /* 函数体开始 */
    int idx = seq % FEC_CACHE_SZ; /* 散列槽号 */
    if (s_cache[idx].valid && s_cache[idx].seq == seq) { /* 校验槽有效且序号匹配 */
        return &s_cache[idx]; /* 命中则返回槽指针 */
    } /* 条件结束 */
    return NULL; /* 未命中返回 NULL */
} /* 函数结束 */

static void fec_group_set(uint16_t fec_seq, uint8_t state, uint8_t window) /* 记录 FEC 组状态 (收到校验包后标记) */
{ /* 函数体开始 */
    int slot = fec_seq % FEC_GROUP_REC; /* 环形槽号: 按 FEC 包 seq 取模 */
    s_fec_groups[slot].fec_seq = fec_seq; /* 记录 FEC 包序号 */
    s_fec_groups[slot].state   = state; /* 记录组状态 */
    s_fec_groups[slot].window  = window; /* 记录组大小 (10 / 5) */
    s_fec_groups[slot].valid   = true; /* 标记该记录有效 */
} /* 函数结束 */

/* 查询某个数据包 seq 是否已被某个已收到的 FEC 组覆盖 (已过恢复期) */
static bool fec_group_covers(uint16_t seq, uint8_t *state_out) /* 查询某数据包 seq 是否已被已收到的 FEC 组覆盖 */
{ /* 函数体开始 */
    for (int i = 0; i < FEC_GROUP_REC; i++) { /* 遍历全部组状态记录 */
        if (!s_fec_groups[i].valid) continue; /* 跳过无效记录 */
        uint16_t d = (uint16_t)(seq - s_fec_groups[i].fec_seq); /* 计算与 FEC 包的相对偏移 (uint16 回绕安全) */
        if (d >= 1 && d <= s_fec_groups[i].window) { /* 落在保护窗口 [1, N] 即被覆盖 */
            *state_out = s_fec_groups[i].state; /* 输出该组的恢复状态供调用方决策 */
            return true; /* 命中返回 true */
        } /* 条件结束 */
    } /* 循环结束 */
    return false; /* 未被任何 FEC 组覆盖 */
} /* 函数结束 */

/* ============ 环形缓冲 (单生产者: net 任务 / 单消费者: i2s 任务) ============ */
typedef struct { /* 环形缓冲结构: 单生产者(net) 单消费者(i2s) */
    uint8_t            *buf; /* 缓冲区指针 (heap_caps_malloc 分配) */
    uint32_t            cap;        /* 字节容量 */
    uint32_t            rd, wr;     /* 读写字节指针 */
    uint32_t            cnt;        /* 已用字节 */
    uint32_t            pkt_bytes;  /* 单包字节 */
    SemaphoreHandle_t   lock; /* 互斥锁: 保护读写指针 */
} ring_t; /* 环形缓冲类型定义结束 */

static ring_t s_ring; /* 全局环形缓冲实例 */

static void ring_init(uint32_t cap, uint32_t pkt_bytes) /* 环形缓冲初始化 */
{ /* 函数体开始 */
    /* 64 包 × 1152B ≈ 74KB, 显式放 PSRAM (内部 SRAM 连续空间不足); 音频搬运 ~2.3MB/s, PSRAM 带宽充裕 */
    s_ring.buf       = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); /* 分配缓冲内存 (PSRAM) */
    assert(s_ring.buf); /* 分配失败断言停机 */
    s_ring.cap       = cap; /* 记录容量 (字节) */
    s_ring.rd = s_ring.wr = s_ring.cnt = 0; /* 读写指针与计数清零 */
    s_ring.pkt_bytes = pkt_bytes; /* 记录单包字节数 */
    s_ring.lock      = xSemaphoreCreateMutex(); /* 创建互斥锁 */
    assert(s_ring.lock); /* 锁创建失败断言停机 */
} /* 函数结束 */

static uint32_t ring_pkt_cnt(void) /* 查询当前缓冲区内的包数量 */
{ /* 函数体开始 */
    uint32_t n; /* 包数量变量 */
    xSemaphoreTake(s_ring.lock, portMAX_DELAY); /* 加锁 (阻塞等待) */
    n = s_ring.cnt / s_ring.pkt_bytes; /* 已用字节数除以单包字节得包数 */
    xSemaphoreGive(s_ring.lock); /* 解锁 */
    return n; /* 返回包数 */
} /* 函数结束 */

/* 查询剩余可入队包数: 批量包需一次性确认能放下 B 帧, 避免拆到一半失败造成重复/丢帧 */
static uint32_t ring_room_pkts(void) /* 查询环形缓冲剩余空间 (以 1ms 包计) */
{ /* 函数体开始 */
    uint32_t n; /* 剩余包数 */
    xSemaphoreTake(s_ring.lock, portMAX_DELAY); /* 加锁 */
    n = (s_ring.cap - s_ring.cnt) / s_ring.pkt_bytes; /* 剩余字节 / 单包字节 */
    xSemaphoreGive(s_ring.lock); /* 解锁 */
    return n; /* 返回剩余包数 */
} /* 函数结束 */

/* 生产者入队; 满则丢弃新包 (返回 false) */
static bool ring_push(const uint8_t *pkt) /* 生产者入队: 满则丢弃新包返回 false */
{ /* 函数体开始 */
    bool ok = false; /* 入队结果标志 */
    xSemaphoreTake(s_ring.lock, portMAX_DELAY); /* 加锁 */
    if (s_ring.cnt + s_ring.pkt_bytes <= s_ring.cap) { /* 空间足够才入队 */
        memcpy(s_ring.buf + s_ring.wr, pkt, s_ring.pkt_bytes); /* 拷贝整包到写指针处 */
        s_ring.wr = (s_ring.wr + s_ring.pkt_bytes) % s_ring.cap; /* 写指针按环形推进 */
        s_ring.cnt += s_ring.pkt_bytes; /* 已用字节数累加 */
        ok = true; /* 入队成功 */
    } /* 空间不足分支结束 */
    xSemaphoreGive(s_ring.lock); /* 解锁 */
    return ok; /* 返回入队结果 */
} /* 函数结束 */

/* 消费者出队; 空则返回 false */
static bool ring_pop(uint8_t *pkt) /* 消费者出队: 空则返回 false */
{ /* 函数体开始 */
    bool ok = false; /* 出队结果标志 */
    xSemaphoreTake(s_ring.lock, portMAX_DELAY); /* 加锁 */
    if (s_ring.cnt >= s_ring.pkt_bytes) { /* 至少有一个整包才出队 */
        memcpy(pkt, s_ring.buf + s_ring.rd, s_ring.pkt_bytes); /* 从读指针处拷贝整包 */
        s_ring.rd = (s_ring.rd + s_ring.pkt_bytes) % s_ring.cap; /* 读指针按环形推进 */
        s_ring.cnt -= s_ring.pkt_bytes; /* 已用字节数减少 */
        ok = true; /* 出队成功 */
    } /* 数据不足分支结束 */
    xSemaphoreGive(s_ring.lock); /* 解锁 */
    return ok; /* 返回出队结果 */
} /* 函数结束 */

/* ============ 输出投递 ============ */
static uint8_t s_silence[sizeof(uint32_t) + PAYLOAD_LEN];   /* 静音帧: [ts=0, 192B全零] */

/* STEREO 槽写入: MONO 模式下 DMA 每帧只占 4B, 使实际缓冲深度减半(4ms),
 * 造成频繁欠载; 这里把 mono 样本复制成 L/R 交错后按 STEREO 写入,
 * 每帧 8B, DMA 深度恢复设计值 8ms, 且消耗节奏与 1ms/包严格一致 */
static uint8_t s_stereo_buf[PAYLOAD_LEN * 2];
static void i2s_write_lr(const uint8_t *mono, size_t nbytes)
{
    const int32_t *src = (const int32_t *)mono;
    int32_t *dst = (int32_t *)s_stereo_buf;
    size_t samples = nbytes / sizeof(int32_t);   /* 48 个样本 */
    for (size_t i = 0; i < samples; i++) {
        dst[i * 2]     = src[i];                 /* L */
        dst[i * 2 + 1] = src[i];                 /* R (与 L 相同) */
    }
    size_t w = 0;
    esp_err_t err = i2s_channel_write(s_tx, s_stereo_buf, nbytes * 2, &w, pdMS_TO_TICKS(100));
    if (err == ESP_ERR_TIMEOUT) {
        /* DMA 超过 100ms 没有消耗: 通道异常(时钟停止/描述符卡住).
         * 若用 portMAX_DELAY 会永久阻塞 → i2s 不再取包 → ring 灌满 → 无声.
         * 这里重启通道自愈, 然后让出 CPU 兜底, 保证 IDLE 能跑. */
        ESP_LOGW(TAG, "i2s write stalled, restarting channel");
        i2s_channel_disable(s_tx);
        i2s_channel_enable(s_tx);
        return;
    }
    if (err != ESP_OK || w != nbytes * 2) {
        /* I2S 异常(通道未就绪/DMA 不消耗)时 write 会立即返回而不阻塞.
         * i2s 任务优先级 5, 若此处不阻塞就会空转饿死 IDLE(优先级 0),
         * 5 秒后触发 Task WDT: "IDLE (CPU 0) / CPU 0: i2s".
         * 这里主动让出 1 tick, 兜底保证 IDLE 有机会运行. */
        vTaskDelay(1);
    }
}

/* 调试探针: i2s 最近一次定时判定, 供 stat 日志输出 (定位"无声"是超前死等还是欠载) */
static volatile int32_t  s_dbg_late_ms;
static volatile int32_t  s_dbg_late_min;      /* 统计窗内 late 最小值 (负=超前) */
static volatile int32_t  s_dbg_late_max;      /* 统计窗内 late 最大值 (正=落后) */
static volatile bool     s_dbg_use_wall;      /* 当前定时模式: true=绝对墙钟 / false=本地锚定 */
static volatile int32_t  s_dbg_abs_off_ms;    /* 绝对模式下的纠偏偏移 (ms) */
static volatile uint32_t s_dbg_ring_pkts;
static volatile uint32_t s_dbg_rx_ts;         /* 最近一个数据包的 RTP ts (诊断: 折算发端时刻) */
static volatile uint32_t s_wifi_disc_cnt;     /* 累计断连次数 (link 诊断: 增长=链路不稳) */
static volatile uint8_t  s_wifi_last_reason;  /* 最近一次断连原因码 (wifi_err_reason_t) */

/* 当前绝对墙钟折算为 48kHz RTP 时钟 (32bit, 约 24.8h 回绕);
 * sender/receiver 经 SNTP 对齐后同一时刻值相同, 用于多设备同步定时 */
static uint32_t wall_rtp_now(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)((uint64_t)tv.tv_sec * 48000ULL + (uint64_t)tv.tv_usec * 48ULL / 1000ULL);
}

static volatile bool s_ntp_synced; /* 对时完成锁存 (回调置位; esp_sntp_get_sync_status 是一次性 API, 不能轮询) */

/* SNTP 是否已完成首次对时 (未对时时 gettimeofday 是 1970, 不能做绝对定时)
 * 注意: 不能轮询 esp_sntp_get_sync_status() —— ESP-IDF v6.1 该函数是一次性的,
 * 读到 COMPLETED 后会把内部状态清回 RESET, 再读就是 0, 必须由回调锁存。 */
static bool time_is_synced(void)
{
    return s_ntp_synced;
}

/* 授时诊断信息: 当前 NTP server / 对时完成次数 / 墙钟与发端 RTP ts 的偏差 (ms) */
static char s_sntp_server[32] = "-";       /* 当前对时 server IP (未启动时为 "-") */
static volatile uint32_t s_ntp_sync_cnt;   /* 对时完成次数 (stat 行 syncs=) */
static volatile int32_t s_ntp_offset_ms;   /* 墙钟 - 发端 ts (ms, 正=墙钟超前; 收包时 EMA 更新) */

/* 对时完成回调 (esp_netif_sntp 注册): 打印对时成功后的本地时间与当前偏差 */
static void sntp_sync_cb(struct timeval *tv) /* 同步完成回调: 入参即刚设置的墙钟 */
{ /* 函数体开始 */
    char tstr[32] = "--:--:--"; /* 本地时间字符串 (格式化失败时保留占位) */
    struct tm tm_now; /* 拆分后的年月日时分秒 */
    time_t sec = tv->tv_sec; /* 刚由 SNTP 写入的墙钟秒数 */
    if (localtime_r(&sec, &tm_now) != NULL) { /* 转本地时间 (中国时区由 sdkconfig 设定) */
        strftime(tstr, sizeof(tstr), "%Y-%m-%d %H:%M:%S", &tm_now); /* 格式化为可读时间 */
    } /* 转换成功分支结束 */
    s_ntp_sync_cnt++; /* 对时成功次数累加 */
    s_ntp_synced = true; /* 锁存"已对时" (api 状态是一次性的, 不能轮询) */
    ESP_LOGI(TAG, "SNTP 已对齐 #%lu: %s (server=%s, offset=%ldms)", /* 日志: 次数/墙钟/发端/偏差 */
             (unsigned long)s_ntp_sync_cnt, tstr, s_sntp_server, /* 次数, 本地时间, server IP */
             (long)s_ntp_offset_ms); /* 墙钟-发端 ts 偏差 (若还没收到音频包则为 0) */
} /* 函数结束 */

/* 启动/重建 SNTP 对齐: 以发端 IP 为 NTP server (RTP 源 IP 零配置自动跟随);
 * 若已启动过但 server IP 变了, 先 deinit 再 init, 保证 WiFi 重连/发端换 IP 后时基自动跟上 */
static esp_err_t sntp_start_to(const char *server_ip) /* 启动 SNTP, server_ip 用标准 IPv4 点分十进制 */
{ /* 函数体开始 */
    static bool s_sntp_init_done = false; /* esp_netif_sntp_init 只允许一次, 换 server 必须 deinit 再 init */

    if (server_ip == NULL || *server_ip == '\0') return ESP_ERR_INVALID_ARG; /* 入参空 */

    /* 同 IP 已启动: 幂等 (不能读 esp_sntp_get_sync_status 判断, 该 API 会消费状态) */
    if (s_sntp_init_done && strcmp(s_sntp_server, server_ip) == 0) {
        return ESP_OK; /* 已对齐或正在对齐, 不需要重建 */
    }
    if (s_sntp_init_done) { /* 换 IP / 强制重建: deinit */
        esp_netif_sntp_deinit(); /* 释放 SNTP 资源 */
        strcpy(s_sntp_server, "-"); /* 旧 server 失效, 重置诊断字段 */
        s_ntp_offset_ms = 0; /* 偏差随 server 变化, 清零重测 */
        s_ntp_synced = false; /* 换 server 后需重新等待对时 (起播门控会重新计时) */
        s_sntp_init_done = false; /* 重置标记 */
    }

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(server_ip); /* 单 server 配置 (方案 §4.1 只有一个内网发端) */
    cfg.smooth_sync = false; /* 方案 A §F1 坑 2: 必须 false —— 1970 起点的钟靠 µs 级缓调永远追不上墙钟, synced 恒 0 */
    cfg.sync_cb = sntp_sync_cb; /* 通过 cfg 注册回调: esp_netif_sntp_init 会用该字段覆盖全局回调 */
    esp_err_t err = esp_netif_sntp_init(&cfg); /* 启动 SNTP */
    if (err == ESP_OK) { /* 成功 */
        sntp_set_sync_interval(NTP_SYNC_INTERVAL_MS); /* 稳态刷新周期 20s (kconfig 默认 1h 太大) */
        sntp_restart(); /* 立即生效并马上发首包, 不等 STARTUP_DELAY */
        s_sntp_init_done = true; /* 标记已 init */
        strncpy(s_sntp_server, server_ip, sizeof(s_sntp_server) - 1); /* 缓存 server IP (供 stat/回调打印) */
        s_sntp_server[sizeof(s_sntp_server) - 1] = '\0'; /* 保证字符串终止 */
        ESP_LOGI(TAG, "SNTP 启动 (server=%s), 与发端对齐", server_ip); /* 日志: 启动成功 */
    } else { /* 失败 */
        ESP_LOGE(TAG, "SNTP 启动失败 err=0x%x", err); /* 日志: 启动失败码 */
    }
    return err; /* 返回状态 */
} /* 函数结束 */

/* 发送端 ts 是否真的是"墙钟×48"?
 * 协议里 ts 是保留字段, 实测 PC 发送端填的是"开机以来的采样计数"(首包 ts=4.05e8≈2.3h),
 * 与 SNTP 墙钟毫无关系 → 必须回退本地锚定, 否则 late 恒为几十分钟, 所有包都被判迟到丢弃. */
static bool ts_looks_wall(uint32_t ts)
{
    if (!time_is_synced()) return false; /* SNTP 未同步: 墙钟不可信 */
    int32_t gap = (int32_t)(wall_rtp_now() - ts); /* 32bit 有符号差值, 自动处理回绕 */
    return gap < TS_SANE_MS * 48 && gap > -TS_SANE_MS * 48;
}

static bool deliver_silence(uint32_t ts) /* 向环形缓冲推入 1ms 静音 (带合成 ts); 返回 false=ring 满 */
{ /* 函数体开始 */
    memcpy(s_silence, &ts, sizeof(uint32_t)); /* 静音包也带时间戳: 否则不推进时间轴,
                                               * 补 N 个静音后真实包会被判"迟到 N ms"成片丢弃 → 卡顿/没声 */
    if (!ring_push(s_silence)) return false; /* 静音包入队 (失败表示 ring 满, 调用方必须停止推进) */
    s_last_ts = ts; /* 时间轴前进 */
    return true; /* 入队成功 */
} /* 函数结束 */

/* 单播单声道流: 从缓存包取出 ts + payload, 一起 push 到 ring 供 i2s_task 绝对墙钟定时;
 * 批量包 (B=2~7) 按 1ms 拆帧入环, 帧 ts = 包 ts + 帧序 x 48, due 调度逐帧判定 */
static bool deliver_pkt(pkt_cache_t *p) /* 将缓存包交付给环形缓冲; 返回 false=ring 满未入队 */
{ /* 函数体开始 */
    uint8_t buf[sizeof(uint32_t) + PAYLOAD_LEN]; /* [ts(4B) + 1ms payload(192B)] */
    int frames = (p->len > 0) ? (int)(p->len / PAYLOAD_LEN) : 0; /* 本包含几个 1ms 帧 */
    if (frames <= 0) return false; /* 长度异常: 不入队 */
    if (ring_room_pkts() < (uint32_t)frames) return false; /* 空间不足整包放下: 等 i2s 消费, 不拆半包 */
    for (int i = 0; i < frames; i++) { /* 逐帧交付 */
        uint32_t fts = p->ts + (uint32_t)i * 48u; /* 帧 ts: 包首帧墙钟 + 帧序 x 48 */
        memcpy(buf, &fts, sizeof(fts)); /* 写入帧 ts */
        memcpy(buf + sizeof(fts), p->data + i * PAYLOAD_LEN, PAYLOAD_LEN); /* 拷贝该 1ms 载荷 */
        if (!ring_push(buf)) return false; /* 入队失败: 停止 (空间已预检, 正常不会发生) */
        s_last_ts = fts; /* 时间轴推进到该帧 */
    } /* 帧循环结束 */
    return true; /* 整包入队成功 */
} /* 函数结束 */

/* ============ FEC 处理 ============ */
/* 收到 FEC 包: fec_seq=F, 保护数据包 seq 落在 [F+1, F+n] (uint16 回绕);
 * n=10 (pt=97, 1ms 流) 或 n=5 (pt=101, 批量流);
 * 批量流载荷 B x 192B, 包间 ts 增量 = 48 x B */
static void handle_fec(uint16_t fec_seq, const uint8_t *fec_data, uint16_t fec_len, uint8_t n) /* 处理 FEC 校验包: 异或还原窗口内丢失的 1 个数据包 */
{ /* 函数体开始 */
    uint8_t xor_buf[BATCH_MAX_PAYLOAD]; /* 异或累加缓冲区 (最大 1344 字节) */
    uint8_t b = (uint8_t)(fec_len / PAYLOAD_LEN); /* 每包毫秒数: 1=1ms 流 / 2~7=批量流 */
    if (b == 0) return; /* 长度异常: 忽略 */
    memset(xor_buf, 0, fec_len); /* 异或缓冲清零 */
    int      cnt     = 0; /* 已收集到的数据包计数 */
    uint16_t missing = 0; /* 记录缺失包的 seq (缺 1 包时用于恢复) */
    uint16_t known_seq = 0; /* 已知包 seq (用于推算缺失包 ts) */
    uint32_t known_ts  = 0; /* 已知包 ts */

    for (int i = 1; i <= n; i++) { /* 遍历保护窗口 [fec_seq+1, fec_seq+n] */
        uint16_t seq = (uint16_t)(fec_seq + i); /* 计算窗口内数据包 seq */
        pkt_cache_t *p = cache_get(seq); /* 从缓存查询该包 */
        if (p && p->len == fec_len) { /* 包在缓存中且长度匹配同一流 */
            for (int j = 0; j < fec_len; j++) xor_buf[j] ^= p->data[j]; /* 逐字节异或累加, 用于还原缺失包 */
            cnt++; /* 计数加一 */
            known_seq = seq; known_ts = p->ts; /* 记录一个已知包, 恢复时推算 ts */
        } else { /* 包缺失 (或长度不等于 FEC 组, 视为不同流) */
            missing = seq; /* 记录缺失包序号 */
        } /* 缺失分支结束 */
    } /* 窗口循环结束 */

    if (cnt == n) { /* 情况1: 窗口内全部收到, 无需恢复 */
        fec_group_set(fec_seq, 1, n);        /* 冗余包, 无丢包 */
    } else if (cnt == n - 1) { /* 情况2: 恰好缺 1 包, 用 FEC 校验包异或还原 */
        for (int j = 0; j < fec_len; j++) xor_buf[j] ^= fec_data[j]; /* 校验包逐字节异或, 还原出缺失包内容 */
        /* 推算缺失包的 ts: 1ms 流每包差 48, 批量流每包差 48 x B */
        uint32_t recovered_ts = known_ts + (uint32_t)seq_diff(missing, known_seq) * (48u * b);
        cache_put(missing, recovered_ts, xor_buf, fec_len); /* 恢复缺失包入缓存 (带推算 ts) */
        fec_group_set(fec_seq, 1, n); /* 记录该 FEC 组已处理 (防止后续按缺包重复静音) */
        s_stats.recovered++; /* 恢复成功计数加一 */
        /* 不逐包打印日志! 115200 波特下每行阻塞 UART ~5ms, 密集恢复时日志本身
         * 会堵死收包循环造成二次突发丢包 (恢复/坏组日志曾在 100ms 内连打 10 行) */
    } else { /* 情况2 结束 */
        fec_group_set(fec_seq, 2, n);        /* 缺 >1 包, 无法恢复, 缺口将静音 */
        s_stats.fec_bad++; /* 坏组数计入汇总, 由 stat 任务每 5 秒统一输出 */
    } /* 情况3 结束 */
} /* 函数结束 */

/* ============ 按序投递 (drain) ============ */
/* 补 n 个 1ms 静音帧 (批量流缺 1 包 = 缺 B ms); 空间不足返回 false */
static bool deliver_silence_n(int n) /* 连续补 n 个 1ms 静音, ts 单调推进 */
{ /* 函数体开始 */
    if (ring_room_pkts() < (uint32_t)n) return false; /* 空间不足整段放下: 不拆半段 */
    for (int i = 0; i < n; i++) { /* 逐帧入队 */
        if (!deliver_silence(s_last_ts + 48)) return false; /* 入队失败即停 */
    } /* 循环结束 */
    return true; /* 全部入队成功 */
} /* 函数结束 */

static void drain_pkts(void) /* 排水: 尽量按 seq 顺序从缓存交付可连续播放的包 */
{ /* 函数体开始 */
    int sil = 0; /* 本次 drain 已插入的静音段数, 限制单次最多补 1 个 ring 深度,
                    防止 seq 大跳变时在收包循环里空转几万个循环 (曾观察到单次 +6.6 万) */
    const int frames = (s_batch_b >= 1 && s_batch_b <= BATCH_MAX_B) ? s_batch_b : 1; /* 每包毫秒数 (缺 1 包补 frames ms) */
    while (1) { /* 持续循环直到无包可排 */
        if (!s_has_last) { /* 尚未确定播放起点: 先找缓存中最小 seq */
            /* 起始: 找缓存中最旧 seq 作为播放起点 */
            int      found   = -1; /* 最小包槽下标 (-1 表示未找到) */
            uint16_t min_seq = 0; /* 最小 seq 值 */
            for (int i = 0; i < FEC_CACHE_SZ; i++) { /* 遍历全部缓存槽找最小 seq */
                if (s_cache[i].valid && (found < 0 || seq_lt(s_cache[i].seq, min_seq))) { /* 找到有效且更小的包则更新候选 */
                    found   = i; /* 记录槽下标 */
                    min_seq = s_cache[i].seq; /* 记录最小 seq */
                } /* 条件结束 */
            } /* 缓存无有效包, 停止排水 */
            if (found < 0) break; /* 确定起点: 播放在最小 seq 前一个位置 (等待下一个连续包) */
            s_last_played = (uint16_t)(min_seq - 1); /* 设置已播放 seq = min_seq - 1 */
            s_has_last    = true; /* 标记播放起点已确定 */
            continue; /* 回到循环重新排水 */
        } /* 起点未确定分支结束 */

        uint16_t next = (uint16_t)(s_last_played + 1); /* 计算期望播放下一个包的 seq */
        pkt_cache_t *p = cache_get(next); /* 从缓存取下一个连续包 */
        if (p) { /* 命中: 可连续播放 */
            if (!deliver_pkt(p)) { s_stats.dropped++; break; } /* ring 满: 停止排水等 i2s 消耗, 不能推进进度(否则音频被静默跳过) */
            s_last_played = next; /* 已入队才算播出, 推进已播放 seq */
            continue; /* 继续排水 */
        } /* 命中分支结束 */

        /* 缺失: 是否已过 FEC 恢复期? */
        uint8_t fstate = 0; /* 下一包缺失: 查询是否被已收 FEC 组覆盖 */
        if (fec_group_covers(next, &fstate)) { /* 已收 FEC 组覆盖该 seq (包确实发出但本机缺失) */
            /* 该缺口所在 FEC 组已收到: 若已恢复成功则上面 cache_get 会命中;
             * 到这里说明原包丢失且未能恢复, 静音推进 (批量流补齐 B ms, 不会堆积) */
            if (sil++ >= DRAIN_FILL_MAX) break; /* 单次排水上限: 保证缺包后在几 ms 内追平最新包 */
            if (!deliver_silence_n(frames)) break; /* ring 满则停, 绝不越过未入队的包推进进度 */
            s_last_played = next; /* 静音也推进播放进度 */
            s_stats.missed += (uint32_t)frames; /* 静音缺包计数 (按 ms 计) */
            if (s_batch_b > 1) s_stats.missed_batch += (uint32_t)frames; /* 批量流单独统计 */
            continue; /* 继续排水 */
        } /* FEC 覆盖分支结束 */

        /* 超时保护: next 落后于缓存最新包超过 FEC 组大小, 说明永远等不到, 静音推进 */
        const int fec_win = (s_batch_b > 1) ? FEC_N_BATCH : FEC_N; /* 批量流恢复窗口按 5 包算 */
        uint16_t newest = 0; /* 下一包缺失且未被 FEC 覆盖: 判断是否已落后过多 */
        bool     has_new = false; /* 缓存中最新的 seq */
        for (int i = 0; i < FEC_CACHE_SZ; i++) { /* 是否找到有效包 */
            if (s_cache[i].valid && (!has_new || seq_gt(s_cache[i].seq, newest))) { /* 遍历缓存找最大 seq */
                newest   = s_cache[i].seq; /* 更新最新 seq */
                has_new  = true; /* 标记找到 */
            } /* 条件结束 */
        } /* 循环结束 */
        if (has_new) { /* 找到缓存最新包才做距离判断 */
            int16_t back = seq_diff(newest, next); /* 最新包相对期望包的有符号距离 */
            if (back < -2000) { /* 最新 seq 反而落后期望 seq 2000 以上: 发送端重启/换曲, seq 回绕到 0 */
                /* 清空旧流缓存与 FEC 组状态, 重新锚定到新流起点;
                 * 否则会沿旧 seq 插几万包静音 (日志实测单次 +6.6 万 miss) */
                memset(s_cache, 0, FEC_CACHE_SZ * sizeof(pkt_cache_t)); /* 清包缓存 (PSRAM) */
                memset(s_fec_groups, 0, sizeof(s_fec_groups)); /* 清 FEC 组记录 */
                s_has_last = false; /* 强制下次 drain 重新找起点 */
                s_last_ts  = 0; /* 新流时间轴从头开始 */
                ESP_LOGW(TAG, "sender stream restart detected (seq jumped back %d), resyncing", (int)back); /* 罕见事件, 允许打印 */
                break; /* 等新流数据包到达后重新锚定 */
            } /* 流重启分支结束 */
            if (back > fec_win) { /* 最新包已超过期望包一个 FEC 组以上: 确认该包已永远丢失 */
                if (back > GAP_JUMP_PKTS) { /* 缺口太大: 逐包补静音会把播放永久推后几十 ms, 直接跳时间轴 */
                    s_stats.missed += (uint32_t)(back - 1) * (uint32_t)frames; /* 缺口计入缺包统计 (按 ms) */
                    if (s_batch_b > 1) s_stats.missed_batch += (uint32_t)(back - 1) * (uint32_t)frames; /* 批量流单列 */
                    s_last_played = (uint16_t)(newest - 1); /* 跳到最新包前一位, 下一轮即可命中 */
                    ESP_LOGW(TAG, "seq gap too large (%d pkt), skip to newest %u", (int)back, (unsigned)newest); /* 罕见事件, 允许打印 */
                    continue; /* 重新排水 (next=newest 应从缓存命中) */
                } /* 大缺口跳转分支结束 */
                if (sil++ >= DRAIN_FILL_MAX) break; /* 单次排水上限: 几 ms 内追平, 不长时间占用收包任务 */
                if (!deliver_silence_n(frames)) break; /* ring 满则停, 等 i2s 消耗 */
                s_last_played = next; /* 推进播放进度 */
                s_stats.missed += (uint32_t)frames; /* 缺包计数 (按 ms 计) */
                if (s_batch_b > 1) s_stats.missed_batch += (uint32_t)frames; /* 批量流单独统计 */
                continue; /* 继续排水 */
            } /* 落后过多分支结束 */
        } /* 有效最新包判断结束 */

        break;   /* 等待后续数据包 / FEC 包 */
    } /* 循环结束 (无包可排) */
} /* 函数结束 */

/* ============ WiFi STA (5GHz) ============ */

/* ---------- WS2812B 状态灯: 配网未连接蓝绿交替闪 / BLE已连接蓝色1Hz闪 / WiFi重连黄灯闪 / 配网完成常绿 ---------- */
#define LED_GPIO_WS2812   27                  /* WS2812B 数据脚 (GPIO27) */
#define LED_BG_HALF_MS    250                 /* 蓝绿交替: 每 250ms 切换 (周期 0.5s) */
#define LED_BLUE_HALF_MS  500                 /* 连接后蓝色闪烁: 亮 500ms 灭 500ms (1Hz) */
#define LED_YELLOW_HALF_MS 500               /* WiFi 重连黄灯闪烁: 亮 500ms 灭 500ms (1Hz) */

typedef enum { /* 状态灯状态机 */
    LED_ST_BG_BLINK = 0, /* 配网中, 等待手机连接: 蓝绿交替闪 (0.5s 周期) */
    LED_ST_BLUE_BLINK,   /* BLE 已连接: 蓝色 1Hz 闪烁 */
    LED_ST_SOLID_GREEN,  /* 配网完成 / 工作模式: 常绿 */
    LED_ST_WIFI_RECONNECT, /* WiFi 未连上 / 中途掉线: 黄灯 1Hz 闪烁 */
} led_state_t; /* 枚举结束 */

static volatile led_state_t s_led_state = LED_ST_SOLID_GREEN; /* 当前灯状态 (默认常绿) */
static volatile led_state_t s_led_base  = LED_ST_SOLID_GREEN; /* 未连接时的基准状态: 配网=蓝绿交替, 工作=常绿 */

/* WiFi/IP 事件回调前向声明 (定义在本文件后半段, 此处为 wifi_sta_init 注册事件所需) */
static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

static const char *const WIFI_SLOT_SUFFIX[8] = { "L", "R", "C", "LFE", "SL", "SR", "SBL", "SBR" }; /* WiFi hostname 声道后缀表, 与 BLE 一致 */
#define WIFI_HOSTNAME_BASE  "7CH-RECEIVER" /* WiFi 设备名基础部分 (不可变) */

/* 等待 802.11 关联成功: 每 100ms 轮询一次, 超时返回 false
 * 关联成功日志带信道号, 便于确认实际跑在 5GHz (ch>=36) 还是 2.4GHz (ch<=14) */
static bool wifi_wait_assoc(int timeout_ms) /* 返回 true=已关联 */
{ /* 函数体开始 */
    for (int waited = 0; waited < timeout_ms; waited += 100) { /* 按 100ms 步进等待 */
        wifi_ap_record_t ap; /* AP 信息记录 */
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) { /* 能取到 AP 信息说明已关联 */
            ESP_LOGI(TAG, "STA associated to '%s' rssi=%d ch=%d (%s)", ap.ssid, ap.rssi, ap.primary,
                     (ap.primary >= 36) ? "5.8GHz ✅" : "2.4GHz ⚠️ (unexpected)"); /* 信道号直接反映频段 */
            return true; /* 关联成功 */
        } /* 关联成功分支结束 */
        vTaskDelay(pdMS_TO_TICKS(100)); /* 每 100ms 轮询一次 */
    } /* 等待循环结束 */
    return false; /* 超时未关联 */
} /* 函数结束 */

static void wifi_sta_init(const prov_config_t *cfg) /* WiFi STA 初始化与连接 (固定 5.8GHz) */
{ /* 函数体开始 */
    /* 创建默认 STA 网络接口 + 在 start 前设置 hostname (DHCP DISCOVER 携带) */
    esp_netif_create_default_wifi_sta(); /* 创建默认 STA 网络接口 */

    /* 注册 WiFi/IP 事件回调: 处理启动时首次连接 + 运行中途掉线重连 + 黄灯闪烁指示 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ip_event_handler, NULL, NULL)); /* WiFi 事件 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_ip_event_handler, NULL, NULL)); /* GOT_IP */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_ip_event_handler, NULL, NULL)); /* LOST_IP */

    /* 在 WiFi 启动前设置 hostname, DHCP DISCOVER 会携带该名, 路由器设备列表正确显示;
     * 命名规则与 BLE 广播名完全一致, 始终带 ID: single 模式 -> "7CH-RECEIVER-<id>-<声道>";
     * 即使 id=0 也显示 "7CH-RECEIVER-0-L" (便于确认 ID 已生效);
     * full(TDM 8ch) 模式无单声道概念 -> "7CH-RECEIVER-<id>" */
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); /* 拿到 STA netif 句柄 */
    if (sta_netif) { /* netif 句柄有效 */
        char hostname[32]; /* hostname 缓冲 (esp_netif_set_hostname 限制 32 字节) */
        if (!s_full && s_slot_index <= 7) { /* single 模式且 slot_index 合法 */
            snprintf(hostname, sizeof(hostname), "%s-%u-%s", WIFI_HOSTNAME_BASE, (unsigned)s_cfg.device_id, WIFI_SLOT_SUFFIX[s_slot_index]); /* "7CH-RECEIVER-0-L" / "7CH-RECEIVER-1-L" */
        } else { /* full 模式 / 未配置: 基础名 + ID */
            snprintf(hostname, sizeof(hostname), "%s-%u", WIFI_HOSTNAME_BASE, (unsigned)s_cfg.device_id); /* "7CH-RECEIVER-0" */
        } /* 后缀分支结束 */
        ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, hostname)); /* 应用 hostname */
        ESP_LOGI(TAG, "WiFi hostname set to '%s' (device_id=%u slot=%u full=%d)", hostname, s_cfg.device_id, s_slot_index, s_full); /* 日志: 确认 hostname 及来源参数 */
    } /* netif 有效分支结束 */

    wifi_init_config_t cfg_net = WIFI_INIT_CONFIG_DEFAULT(); /* 默认 WiFi 初始化配置 */
    ESP_ERROR_CHECK(esp_wifi_init(&cfg_net)); /* 初始化 WiFi 驱动 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); /* 设置 STA 模式 (设置频段模式前必须先设模式) */

    ESP_ERROR_CHECK(esp_wifi_start()); /* 启动 WiFi */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); /* 关闭省电模式: DTIM 休眠会丢包, 音频流接收必须常醒 */
    ESP_LOGI(TAG, "WiFi power save disabled (WIFI_PS_NONE) for UDP RX"); /* 日志确认省电已关 */

    /* ============================================================
     * 频段策略: 固定 5.8GHz (WIFI_BAND_MODE_5G_ONLY)
     *   - ESP32-C5 天线调谐对 5G 优化; 5.8GHz 同频干扰少、带宽大, 适合 48kHz 32bit 音频流
     *   - 不再回退 2.4GHz; 路由器不支持 5GHz 时直接在 5G 上重试
     *   - esp_wifi_set_band_mode 必须在 start 之后调用
     * ============================================================ */
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)); /* 锁定 5GHz */
    wifi_band_mode_t bm = 0; /* 回读缓冲 */
    if (esp_wifi_get_band_mode(&bm) == ESP_OK) { /* 回读校验实际生效的频段模式 */
        ESP_LOGI(TAG, "WiFi band mode=%d (2=5G only ✅, fixed 5.8GHz)", (int)bm); /* 确认固定 5G 已生效 */
    } /* 回读结束 */

    /* STA 配置放在设定频段之后应用: channel=0 (由 set_band_mode 限定扫描范围) */
    wifi_config_t wc = { 0 }; /* STA 配置结构清零 */
    strncpy((char *)wc.sta.ssid, cfg->ssid, sizeof(wc.sta.ssid) - 1); /* 拷贝配网 SSID (留 1 字节给结尾 0) */
    strncpy((char *)wc.sta.password, cfg->password, sizeof(wc.sta.password) - 1); /* 拷贝配网密码 */
    wc.sta.channel = WIFI_5G_CHANNEL; /* 0=自动扫描 5GHz 全频段; 149~165=锁定 5.8GHz 某信道 */
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; /* 要求 WPA2-PSK 认证 */
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc)); /* 应用 STA 配置 */

    esp_wifi_connect(); /* 发起 5.8GHz 连接 */
    if (!wifi_wait_assoc(15000)) { /* 等 15 秒完成 5GHz 关联 */
        ESP_LOGE(TAG, "5.8GHz STA associating timeout, check AP 5GHz enabled + ssid/password; retry on WIFI_EVENT"); /* 超时告警 */
        s_led_state = LED_ST_WIFI_RECONNECT; /* 切黄灯闪, 让事件回调驱动后续重连 */
        /* 注意: 不再回退 2.4GHz; 路由器必须启用 5GHz */
        return; /* 返回, 后续由 WIFI_EVENT_STA_DISCONNECTED 自动重连 */
    } /* 关联超时分支结束 */

    /* 必须等到拿到有效 IP 再返回; 未拿到 IP 时发送端无法单播到本机,
     * 且发现广播也需要真实 IP 才能通告 */
    for (int waited = 0; waited < 150; waited++) { /* 最多等待 15 秒 DHCP */
        esp_netif_ip_info_t ipi; /* IP 信息结构 */
        if (esp_netif_get_ip_info(sta_netif, &ipi) == ESP_OK && ipi.ip.addr != 0) { /* 已拿到非零 IP 地址 */
            ESP_LOGI(TAG, "STA got ip: " IPSTR ", network bring-up complete (5.8GHz)", IP2STR(&ipi.ip)); /* 日志: 网络就绪 */
            return; /* IP 就绪, 之后绑定 5004 端口收单播才有效 */
        } /* IP 就绪分支结束 */
        vTaskDelay(pdMS_TO_TICKS(100)); /* 每 100ms 轮询一次 */
    } /* DHCP 等待循环结束 */
    ESP_LOGE(TAG, "DHCP timeout, no IP address; retry on IP_EVENT callback"); /* DHCP 超时告警 */
    s_led_state = LED_ST_WIFI_RECONNECT; /* 切黄灯闪, 让事件回调驱动后续 IP 获取 */
} /* 函数结束 */

/* ============ 单播接收 socket ============ */
static int udp_sock_open(uint16_t port) /* 创建 UDP socket 并绑定本机端口 (纯单播, 不做组播加入) */
{ /* 函数体开始 */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); /* 创建 UDP socket */
    if (sock < 0) { /* 创建失败 */
        ESP_LOGE(TAG, "socket failed errno=%d", errno); /* 日志输出错误码 */
        return -1; /* 返回 -1 */
    } /* 失败分支结束 */
    int opt = 1; /* SO_REUSEADDR 选项: 允许快速重启后绑定同一端口 */
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); /* 设置地址复用选项 */

    /* 接收缓冲区 64KB: 默认 UDP socket 只能排队约 6 个数据报, 1100 包/秒速率下
     * 任何几毫秒的处理抖动 (日志/XOR) 都会溢出丢包, 且表现为连续整组突发丢失 */
    int rcvbuf = 64 * 1024; /* 期望接收缓冲 64KB (lwIP 会按其上限裁剪) */
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)); /* 放大 socket 接收队列 */

    struct sockaddr_in local = { 0 }; /* 本地地址结构清零 */
    local.sin_family      = AF_INET; /* IPv4 地址族 */
    local.sin_port        = htons(port); /* 绑定端口 (网络序) */
    local.sin_addr.s_addr = htonl(INADDR_ANY); /* 监听任意网卡 (单播发到本机 IP 即接收) */
    if (bind(sock, (struct sockaddr *)&local, sizeof(local)) != 0) { /* 绑定本地端口 */
        ESP_LOGE(TAG, "bind %u failed errno=%d", port, errno); /* 绑定失败日志 */
        close(sock); /* 关闭 socket */
        return -1; /* 返回 -1 */
    } /* 绑定失败分支结束 */

    ESP_LOGI(TAG, "unicast socket bound on %u, waiting for RTP stream", port); /* 日志确认已绑定 */
    return sock; /* 返回 socket 句柄 */
} /* 函数结束 */

/* ============ 网络任务: STA -> 单播 RTP/FEC -> ring ============ */
static void network_task(void *arg) /* 网络接收任务: 收 RTP 数据包/FEC 包并排水到环形缓冲 */
{ /* 任务入口 */
    (void)arg; /* 忽略入参 */

    wifi_sta_init(&s_cfg); /* 初始化 WiFi 并连接配网 AP (事件回调已注册, 超时不阻塞) */

    int sock = -1; /* 单播 socket 句柄: -1 表示未建好 */
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); /* netif 句柄, 用于判断 IP 就绪 */

    /* 外层循环: 等 IP 就绪后创建 socket, socket 出错后重建; 保证 WiFi 掉线重连后自动恢复收流 */
    static bool s_sntp_started = false; /* SNTP 是否已在本 session 启动过 (WiFi 重连后重置, 用新发端 IP 重新对时) */
    while (1) {
        s_sntp_started = false; /* 每次进入外层循环都重置: WiFi 重连后, 新发端 IP 必须触发一次新 SNTP */
        /* 等待有效 IP: 没拿到 IP 就每 1s 轮询一次 (同时事件回调在后台处理重连) */
        while (sock < 0) {
            if (sta_netif) { /* netif 有效 */
                esp_netif_ip_info_t ipi; /* IP 信息结构 */
                if (esp_netif_get_ip_info(sta_netif, &ipi) == ESP_OK && ipi.ip.addr != 0) { /* 已拿到非零 IP */
                    /* 方案 A §F1: SNTP 不再直连 pool.ntp.org, 而是收 RTP 时取发端 IP 作为 server.
                     * 这样音箱与发端同一 NTP 时基, 台间误差由"各自对公网 10~30ms"降到
                     * "对齐 RTP ts 产生者 ±1~3ms" */
                    sock = udp_sock_open(DEF_AUDIO_PORT); /* 绑定固定 UDP 5004 单播端口 */
                    if (sock >= 0) { /* 成功 */
                        ESP_LOGI(TAG, "unicast bind OK, sock=%d, start recv", sock); /* 日志 */
                        break; /* 跳出等待循环, 进入收包 */
                    } /* 成功分支结束 */
                    ESP_LOGE(TAG, "unicast bind failed (sock=%d), retry in 1s...", sock); /* bind 失败 */
                } /* IP 就绪分支结束 */
            } /* netif 有效分支结束 */
            vTaskDelay(pdMS_TO_TICKS(1000)); /* 每 1s 轮询一次 IP */
        } /* 等待 IP + bind 成功循环结束 (此时 sock >= 0) */

        uint8_t pkt[RTP_HDR_LEN + BATCH_MAX_PAYLOAD]; /* 接收缓冲区: RTP 头 + 最大载荷 (批量 1344B, 总长 1356B < 1400) */
        struct sockaddr_in from; /* 发送方地址结构 */
        socklen_t fromlen = sizeof(from); /* 地址结构长度 */

        /* 收包子循环: WiFi 掉线时 recvfrom 会立即返回错误, 此时关闭 socket 回外层等 IP 重建 */
        while (1) { /* 主收包循环 */
            int len = recvfrom(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &fromlen); /* 阻塞接收 UDP 数据报 */
            if (len <= 0) { /* 接收失败 (WiFi 掉线 / 超时 / socket 关闭) */
                ESP_LOGW(TAG, "recvfrom returned %d, closing socket and retrying join...", len); /* 日志: 收包错误, 重建 socket */
                close(sock); /* 关闭旧 socket */
                sock = -1; /* 标记未建好, 外层循环重新等 IP + join */
                break; /* 跳回外层 while(1) */
            } /* 失败分支结束 */
            if (len < (int)RTP_HDR_LEN) { s_stats.rx_short++; continue; } /* 短于 RTP 头: 计数后丢弃 (用于 UDP 可达性测试) */

            rtp_header_t *hdr = (rtp_header_t *)pkt; /* 将缓冲区强转为 RTP 头结构 */
            uint16_t seq  = ntohs(hdr->seq); /* 序号转主机序 */
            uint32_t ssrc = ntohl(hdr->ssrc); /* SSRC 转主机序 */
            if (ssrc != SSRC) {               /* 过滤非本系统流 */
                s_stats.bad_ssrc++; /* 非本系统流: 非法 SSRC 计数加一 */
                continue; /* 丢弃该包 */
            } /* 非法 SSRC 分支结束 */
            int plen = len - (int)RTP_HDR_LEN; /* 载荷长度 */
            if (plen <= 0 || plen > BATCH_MAX_PAYLOAD || (plen % PAYLOAD_LEN) != 0) { /* 非整 1ms 帧的载荷: 丢弃 */
                s_stats.rx_badlen++; /* 长度异常计数 (含 IP 分片重组失败) */
                continue; /* 丢弃该包 */
            } /* 长度检查结束 */
            const uint8_t *payload = pkt + RTP_HDR_LEN; /* 载荷指针指向 RTP 头之后 */

            /* 方案 A §F1-2: 首次收到合法 RTP 包时, 用 RTP 源 IP 启动 SNTP 对齐发端.
             * 运行期若已启动但 RTP 源 IP 变了 (发端换 IP / WiFi 漫游), sntp_start_to
             * 内部会 deinit+init 重建, 保证时基始终跟着 ts 产生者走 */
            if (!s_sntp_started) { /* 本 session 还没启动过 */
                char ip_str[INET_ADDRSTRLEN]; /* 存点分十进制字符串 */
                if (inet_ntoa_r(from.sin_addr, ip_str, sizeof(ip_str)) != NULL) { /* 转换成功 */
                    s_sntp_started = (sntp_start_to(ip_str) == ESP_OK); /* 成功才标记已启动, 下次收包重试 */
                } /* 转换分支结束 */
            } /* SNTP 触发分支结束 */

            if (hdr->pt == RTP_PT || hdr->pt == RTP_PT_BATCH) { /* 数据包 (pt=96 1ms / 100 批量) */
                uint8_t b = (uint8_t)(plen / PAYLOAD_LEN); /* 本包毫秒数: 1 / 2~7 */
                if (hdr->pt == RTP_PT && b != 1) { s_stats.rx_badlen++; continue; } /* pt=96 必须是 192B */
                s_batch_b = b; /* 学习当前流每包毫秒数 (缺包补静音/恢复窗口据此计算) */
                if (hdr->pt == RTP_PT) s_stats.rx_data++; else s_stats.rx_batch++; /* 分别计数 96/100 */
                s_last_rx_us = esp_timer_get_time(); /* 记录最近收流时刻 (发现广播按有无音频流调周期) */
                if (cache_get(seq)) { /* 重复包检测: 该 seq 已在缓存中 */
                    s_stats.seq_dup++; /* 重复包计数加一 */
                    continue; /* 丢弃重复包 */
                } /* 重复分支结束 */
                uint32_t ts = ntohl(hdr->ts); /* sender 的 RTP ts (wall_ms × 48) */
                s_dbg_rx_ts = ts; /* 记录发端 ts, 供 stat 任务折算成发端时刻打印 */
                if (time_is_synced()) { /* 已对时: 采样"墙钟 - 发端 ts", 用于判断发端是否也按 NTP 打时间戳 */
                    int32_t d_ms = (int32_t)(wall_rtp_now() - ts) / 48; /* ms, 32bit 差值自动处理回绕 */
                    s_ntp_offset_ms = (d_ms + 3 * s_ntp_offset_ms) / 4; /* EMA 平滑, 抑制单包网络抖动 */
                } /* 对时采样分支结束 */
                cache_put(seq, ts, payload, (uint16_t)plen); /* 新包写入缓存 (带 ts 供绝对墙钟定时) */
                drain_pkts(); /* 触发排水: 按 seq 交付, 批量包内部拆成 1ms 帧 */
            } else if (hdr->pt == RTP_PT_FEC || hdr->pt == RTP_PT_BATCH_FEC) { /* FEC 校验包 (97 1ms / 101 批量) */
                bool batch_fec = (hdr->pt == RTP_PT_BATCH_FEC); /* 是否批量流 FEC */
                if (!batch_fec && plen != PAYLOAD_LEN) { s_stats.rx_badlen++; continue; } /* pt=97 必须是 192B */
                if (batch_fec) s_stats.rx_batch_fec++; else s_stats.rx_fec++; /* 分别计数 97/101 */
                handle_fec(seq, payload, (uint16_t)plen, batch_fec ? FEC_N_BATCH : FEC_N); /* 窗口: 批量 N=5 / 1ms N=10 */
                drain_pkts(); /* 尝试异或恢复缺失数据包 */
            } /* 恢复后再次排水 */
        } /* 收包循环结束 */
    } /* 外层 while(1): 等 IP + join + 收包 / 重连 */
} /* 函数结束 */

/* ============ 统计任务: 每 5 秒无条件打印一次计数 (无包时也可见) ============ */
static void stat_task(void *arg) /* 运行统计打印任务: 独立于收包循环, 诊断网络静默问题 */
{ /* 任务入口 */
    (void)arg; /* 忽略入参 */
    while (1) { /* 打印主循环 */
        vTaskDelay(pdMS_TO_TICKS(5000)); /* 每 5 秒打印一次 */
        bool synced = time_is_synced(); /* 方案 A §F3: 对时状态 */
        char wall[16] = "--:--:--"; /* 本地墙钟 HH:MM:SS (未对时显示占位, 避免 1970 误导) */
        long wall_ms = 0; /* 本地墙钟毫秒部分 */
        if (synced) { /* 已对时: 格式化当前墙钟 */
            struct timeval tv; /* 当前墙钟 */
            struct tm tm_now; /* 拆分后的时分秒 */
            gettimeofday(&tv, NULL); /* 取 NTP 时间 */
            wall_ms = (long)(tv.tv_usec / 1000); /* 毫秒 */
            if (localtime_r(&tv.tv_sec, &tm_now) != NULL) { /* 转本地时间 */
                strftime(wall, sizeof(wall), "%H:%M:%S", &tm_now); /* 只打印时分秒, 行长可控 */
            } /* 转换成功分支结束 */
        } /* 对时分支结束 */
        ESP_LOGI(TAG, "stat: rx=%lu fec=%lu rec=%lu fecbad=%lu miss=%lu late_drop=%lu jump=%lu fix=%lu drop=%lu under=%lu bad_ssrc=%lu short=%lu badlen=%lu pt100=%lu/%lu miss100=%lu late=%ld/%ld/%ldms ring=%lu mode=%s synced=%d ntp=%s syncs=%lu wall=%s.%03ld off=%ldms abs_off=%ldms", /* 日志: 收/校/恢/坏组/缺/落后丢/跳变/强制重建/丢/欠载/非法SSRC/短包/长度异常/批量收包/批量FEC/批量缺帧/late(末/最小/最大)/ring/定时模式/对时状态/授时server/对时次数/墙钟(含毫秒)/偏差/绝对纠偏 */
                 (unsigned long)s_stats.rx_data, (unsigned long)s_stats.rx_fec, /* 数据包与 FEC 包计数 */
                 (unsigned long)s_stats.recovered, (unsigned long)s_stats.fec_bad, /* 恢复数与不可恢复组数 */
                 (unsigned long)s_stats.missed, /* 静音补缺计数 (ms) */
                 (unsigned long)s_stats.late_drop, (unsigned long)s_stats.late_jump, /* 时间戳落后丢弃 / 时基跳变 */
                 (unsigned long)s_stats.late_fix, /* 严重滞后强制重建次数 */
                 (unsigned long)s_stats.dropped, (unsigned long)s_stats.underrun, /* 丢弃与欠载计数 */
                 (unsigned long)s_stats.bad_ssrc, /* SSRC 不符丢弃计数 */
                 (unsigned long)s_stats.rx_short, (unsigned long)s_stats.rx_badlen, /* 短包与长度异常计数 */
                 (unsigned long)s_stats.rx_batch, (unsigned long)s_stats.rx_batch_fec, /* 批量数据包 / 批量 FEC 包 */
                 (unsigned long)s_stats.missed_batch, /* 批量流补静音帧数 */
                 (long)s_dbg_late_ms, (long)s_dbg_late_min, (long)s_dbg_late_max, /* late 末值/窗内最小/窗内最大 */
                 (unsigned long)ring_pkt_cnt(), s_dbg_use_wall ? "wall" : "anchor", /* ring 深度 / 当前定时模式 */
                 (int)synced, s_sntp_server, (unsigned long)s_ntp_sync_cnt, /* 对时状态 / 当前 server / 成功次数 */
                wall, wall_ms, (long)s_ntp_offset_ms, (long)s_dbg_abs_off_ms); /* 墙钟 HH:MM:SS.mmm / 墙钟-发端 ts / 绝对纠偏 */
        /* 时间对照: 本地墙钟 vs 发端 RTP ts 折算的时刻 (直观判断两侧时基是否一致) */
        { /* 独立作用域, 避免污染统计打印变量 */
            struct timeval tv; /* 当前本地墙钟 (含毫秒) */
            struct tm tm_loc; /* 本地时间拆分 */
            char loc[32] = "--"; /* 本地时间字符串 (未对时保持占位) */
            char snd[24] = "--:--:--"; /* 发端时刻字符串 (需容纳 "YYYY-MM-DD HH:MM:SS") */
            long snd_ms = 0; /* 发端时刻毫秒部分 */
            unsigned long rts = (unsigned long)s_dbg_rx_ts; /* 最近一包的发端 ts */
            gettimeofday(&tv, NULL); /* 取本地墙钟 */
            if (synced && localtime_r(&tv.tv_sec, &tm_loc) != NULL) { /* 已对时才有意义 */
                strftime(loc, sizeof(loc), "%Y-%m-%d %H:%M:%S", &tm_loc); /* 完整日期时间 */
            } /* 本地时间格式化结束 */
            if (rts != 0 && synced) { /* 已收包且已对时: 32bit ts 就近还原成 64 位再折算 */
                int64_t now_ts = (int64_t)tv.tv_sec * 48000 + (int64_t)tv.tv_usec * 48 / 1000; /* 64 位墙钟 tick (不能用 wall_rtp_now, 那个是 32 位回绕值) */
                int32_t d32 = (int32_t)((uint32_t)rts - (uint32_t)now_ts); /* 与当前墙钟的整数差 (±12.4h 内) */
                int64_t snd_ts = now_ts + d32; /* 还原后的 64 位发端时刻 */
                time_t sec = (time_t)(snd_ts / 48000); /* 折算为秒 */
                snd_ms = (long)((snd_ts - (int64_t)sec * 48000) / 48); /* 毫秒部分 */
                struct tm tm_snd; /* 发端时刻拆分 */
                if (localtime_r(&sec, &tm_snd) != NULL) { /* 转换成功 */
                    strftime(snd, sizeof(snd), "%Y-%m-%d %H:%M:%S", &tm_snd); /* 带日期, 可直接和 local 逐位对比 */
                } /* 发端时间格式化结束 */
            } /* 发端时刻折算结束 */
            ESP_LOGI(TAG, "time: local=%s.%03ld snd=%s.%03ld off=%ldms ts=%lu", /* 本地时刻 / 发端时刻 / 两侧偏差 / 原始ts */
                     loc, (long)(tv.tv_usec / 1000), snd, snd_ms, (long)s_ntp_offset_ms, rts);
        } /* 时间对照打印结束 */
        /* 链路健康: WiFi 关联/信号/断连次数与原因/IP/堆余量 (失联问题定位用) */
        { /* 独立作用域 */
            wifi_ap_record_t ap; /* AP 信息 */
            memset(&ap, 0, sizeof(ap)); /* 清零: 未关联时字段无意义 */
            bool assoc = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK); /* 是否已关联 AP (false=WiFi 掉线) */
            esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); /* STA netif */
            esp_netif_ip_info_t ipi; /* 本机 IP/网关/掩码 */
            memset(&ipi, 0, sizeof(ipi)); /* 清零: 无 IP 时显示 0.0.0.0 */
            if (nif != NULL) esp_netif_get_ip_info(nif, &ipi); /* 读取当前 IP 配置 */
            const unsigned char *ip = (const unsigned char *)&ipi.ip.addr; /* 按网络字节序取 4 字节 */
            const unsigned char *gw = (const unsigned char *)&ipi.gw.addr; /* 网关 4 字节 */
            ESP_LOGI(TAG, "link: wifi=%s rssi=%d ch=%u disc=%lu reason=%u ip=%u.%u.%u.%u gw=%u.%u.%u.%u int_heap=%u psram_heap=%u", /* 关联态/信号/信道/断连次数/原因码/IP/网关/内部堆/PSRAM堆 */
                     assoc ? "up" : "DOWN", assoc ? ap.rssi : 0, assoc ? (unsigned)ap.primary : 0u, /* 未关联时信号与信道填 0 */
                     (unsigned long)s_wifi_disc_cnt, (unsigned)s_wifi_last_reason, /* 累计断连次数 + 最近一次原因码 */
                     (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3], /* 本机 IP */
                     (unsigned)gw[0], (unsigned)gw[1], (unsigned)gw[2], (unsigned)gw[3], /* 网关 */
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), /* 内部 SRAM 剩余 */
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM)); /* PSRAM 剩余 */
        } /* 链路健康打印结束 */
        s_dbg_late_min = 0; /* 重置统计窗最小值 */
        s_dbg_late_max = 0; /* 重置统计窗最大值 */
    } /* 打印循环结束 */
} /* 函数结束 */

/* ============ 发现广播: WiFi 就绪后周期性 UDP 广播 本机声道+IP, 供 PC 发送端建立单播 ============ */
#define DISC_PT          98   /* 发现包 PT (区别于 96=数据 / 97=FEC) */
#define DISC_PORT        6000 /* 发现包目标端口 (PC 发送端监听此端口, 与音频端口 5004 分离) */
#define DISC_WAIT_IP_MS  1000 /* 尚未拿到 IP 时的轮询间隔 (尽快响应 WiFi 上线) */
#define DISC_IDLE_MS     5000 /* 无音频流: 每 5s 广播一次 (催促发送端尽快发现本机) */
#define DISC_ACTIVE_MS   20000 /* 有音频流: 每 20s 广播一次 (流已在传, 降低广播开销) */
#define DISC_ACTIVE_HOLD_MS 3000 /* 最近 3s 内收到过音频数据包即视为"有流" */
#define DISC_BURST_N     3    /* IP 变化/首次上线时连发次数 (加速发送端发现) */
#define DISC_BURST_GAP_MS 200 /* 连发包间隔 */
#define DISC_CAPS_BATCH  0x02 /* 发现包 [14] bit1: caps=1 声明支持批量流 PT100/101 (v1.3) */

/* 发现包载荷 (紧跟 12B RTP 头之后): 设备 ID + 声道 bit + 发送使能/能力位 + IPv4 (网络序) */
typedef struct __attribute__((packed)) {
    uint8_t  id;        /* 设备编号 0~255: 由 PC/手机配网 JSON "device_id" 下发, 同网段多台接收端必须各不相同 */
    uint8_t  slot_bit;  /* 本机声道按 bit 表示: 1<<slot_index, slot0..7 -> 0x01/0x02/0x04/.../0x80 */
    uint8_t  tx_enable; /* v1.3 flags: bit0=发送使能 (1=正常发送/0=停止发送到本机), bit1=caps (1=支持批量流) */
    uint32_t ip;        /* 本机 IPv4 地址 (网络序大端, 发送端据此单播音频流) */
} disc_payload_t; /* 7 字节 */

/* 发送使能标志: 工作模式恒为 1; 配网下发新配置后临时置 0 广播"停止发送", 随后重启 */
static volatile uint8_t s_tx_enable = 1;

/* CRC16-CCITT-FALSE (init 0xFFFF, poly 0x1021): 发现包包尾校验 */
static uint16_t crc16_ccitt(const uint8_t *data, int len) /* 计算发现包 CRC16 包尾 */
{ /* 函数体开始 */
    uint16_t crc = 0xFFFF; /* 初始值 */
    for (int i = 0; i < len; i++) { /* 逐字节计算 */
        crc ^= (uint16_t)data[i] << 8; /* 高字节异或 */
        for (int b = 0; b < 8; b++) { /* 逐位移位 */
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1); /* 多项式 0x1021 */
        } /* 位循环结束 */
    } /* 字节循环结束 */
    return crc; /* 返回 16 位校验 */
} /* 函数结束 */

/* 发现包布局 (与音频包同一 RTP 头协议风格):
 *   [0..11]  RTP 头: 0x80, pt=98, seq(递增), ts(开机毫秒), ssrc=0x7C41
 *   [12]     id        设备编号 (配网下发, 0~255)
 *   [13]     slot_bit  本机声道按 bit: 1<<slot_index
 *   [14]     flags      bit0=tx_enable (1=正常发送 / 0=停止发送到本机)
 *                       bit1=caps (1=支持批量流 PT100/101; 本机恒为 1)
 *   [15..18] ip        本机 IPv4 (网络序)
 *   [19..20] CRC16 (CCITT-FALSE, 对 [0..18] 计算, 小端存放)   <- 包尾
 * 总长 21 字节, 限同网段广播 (目标 = ip | ~netmask), UDP 6000 端口 */

/* 组包并发送一个发现包: 供 discovery_task 周期广播与配网"停止发送"通告共用 */
static void disc_send(int sock, const struct sockaddr_in *dst, /* 已建 socket 与广播目标 */
                      uint8_t id, uint8_t slot_bit, uint8_t tx_enable, uint32_t ip, /* 载荷字段 */
                      uint16_t seq) /* RTP 序号 (调用方维护递增) */
{ /* 函数体开始 */
    rtp_header_t h = { 0 }; /* RTP 头 (与音频包同构) */
    h.v_p_x_cc = 0x80; /* RTP v2 */
    h.pt = DISC_PT; /* 发现包 PT=98 */
    h.seq = htons(seq); /* 序号 (网络序) */
    h.ts  = htonl((uint32_t)(esp_timer_get_time() / 1000)); /* 时间戳 = 开机毫秒 */
    h.ssrc = htonl(SSRC); /* 系统标识 */
    disc_payload_t p = { .id = id, .slot_bit = slot_bit, /* 载荷: ID + 声道bit + 能力位 + IP */
                         .tx_enable = (uint8_t)((tx_enable & 0x01u) | DISC_CAPS_BATCH), /* bit0=tx_enable, bit1=caps */
                         .ip = ip }; /* 本机 IPv4 */

    uint8_t pkt[sizeof(rtp_header_t) + sizeof(disc_payload_t) + 2]; /* 包缓冲: 头+载荷+CRC16 = 21B */
    memcpy(pkt, &h, sizeof(h)); /* 填头 */
    memcpy(pkt + sizeof(h), &p, sizeof(p)); /* 填载荷 */
    uint16_t crc = crc16_ccitt(pkt, sizeof(h) + sizeof(p)); /* 计算包尾 CRC (对 [0..18]) */
    pkt[sizeof(h) + sizeof(p)]     = (uint8_t)(crc & 0xFF); /* CRC 低字节 (小端) */
    pkt[sizeof(h) + sizeof(p) + 1] = (uint8_t)((crc >> 8) & 0xFF); /* CRC 高字节 */

    sendto(sock, pkt, sizeof(pkt), 0, (const struct sockaddr *)dst, sizeof(*dst)); /* 广播发送 */
} /* 函数结束 */

static void discovery_task(void *arg) /* 发现广播任务: 每 5s/20s 广播一次本机 ID+声道+IP */
{ /* 任务入口 */
    (void)arg; /* 忽略入参 */

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); /* 创建 UDP socket */
    if (sock < 0) { /* 创建失败 */
        ESP_LOGE(TAG, "discovery socket failed errno=%d", errno); /* 日志 */
        vTaskDelete(NULL); /* 退出任务 */
        return; /* 不再继续 */
    } /* 失败分支结束 */
    int bc = 1; /* 广播使能 */
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc)); /* 允许发广播地址 */

    /* 设备 ID: 配网 JSON "device_id" 下发 (0~255, 持久化到 NVS); 声道按 bit 编码 */
    const uint8_t dev_id   = s_cfg.device_id; /* 配网下发的设备编号 */
    const uint8_t slot_bit = (uint8_t)(1u << (s_slot_index & 7)); /* 声道 bit: slot0->0x01 ... slot7->0x80 */

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); /* STA netif (读取本机 IP/掩码) */
    struct sockaddr_in dst = { 0 }; /* 广播目标地址 */
    dst.sin_family = AF_INET; /* IPv4 */
    dst.sin_port = htons(DISC_PORT); /* 目标端口 6000 */

    uint32_t last_ip = 0; /* 上次广播时的本机 IP (检测 IP 变化触发连发) */
    uint16_t seq = 0; /* 发现包序号 (递增, 供发送端判断新鲜度) */
    int last_period_ms = 0; /* 上次使用的广播周期 (0=未定), 仅在周期变化时打日志 */

    while (1) { /* 主循环 */
        esp_netif_ip_info_t ipi = { 0 }; /* IP 信息 */
        uint32_t cur = 0; /* 当前本机 IP */
        if (netif && esp_netif_get_ip_info(netif, &ipi) == ESP_OK) cur = ipi.ip.addr; /* 读取当前 IP */
        if (cur == 0) { /* 尚未拿到 IP (WiFi 未连/重连中, 黄灯由事件回调负责) */
            vTaskDelay(pdMS_TO_TICKS(DISC_WAIT_IP_MS)); /* 每 1s 查一次 IP */
            continue; /* 跳过广播 */
        } /* 无 IP 分支结束 */

        /* 目标地址: 同网段定向广播 ip | ~netmask; 掩码无效时自然退化为 255.255.255.255 */
        dst.sin_addr.s_addr = cur | ~ipi.netmask.addr;

        int burst = (cur != last_ip) ? DISC_BURST_N : 1; /* IP 变化/首次上线: 连发 3 包加速发现 */
        bool first = (cur != last_ip); /* 本轮是否 IP 变化 (用于打一次状态日志) */
        last_ip = cur; /* 记录本次 IP */

        for (int i = 0; i < burst; i++) { /* 连发循环 */
            disc_send(sock, &dst, dev_id, slot_bit, s_tx_enable, cur, seq++); /* 组包发送 (tx_enable 工作模式恒 1) */
            if (i + 1 < burst) vTaskDelay(pdMS_TO_TICKS(DISC_BURST_GAP_MS)); /* 连发包间隔 */
        } /* 连发循环结束 */

        if (first) { /* IP 变化/首次上线: 打印一次发现参数便于验证 */
            ESP_LOGI(TAG, "discovery: id=%u slot_bit=0x%02X flags=0x%02X ip=%u.%u.%u.%u bcast=%u.%u.%u.%u:%u", /* 日志 */
                     dev_id, slot_bit, (unsigned)((s_tx_enable & 0x01u) | DISC_CAPS_BATCH), /* 设备编号 / 声道 bit / tx_enable|caps */
                     (unsigned)cur & 0xFF, (unsigned)(cur >> 8) & 0xFF, /* IP 前两字节 */
                     (unsigned)(cur >> 16) & 0xFF, (unsigned)(cur >> 24) & 0xFF, /* IP 后两字节 */
                     (unsigned)dst.sin_addr.s_addr & 0xFF, (unsigned)(dst.sin_addr.s_addr >> 8) & 0xFF, /* 广播地址前两字节 */
                     (unsigned)(dst.sin_addr.s_addr >> 16) & 0xFF, (unsigned)(dst.sin_addr.s_addr >> 24) & 0xFF, /* 广播地址后两字节 */
                     DISC_PORT); /* 目标端口 */
        } /* 日志分支结束 */

        /* 周期自适应: 最近 DISC_ACTIVE_HOLD_MS 内收到过音频数据包 -> 20s; 否则(无流/流中断) -> 5s */
        bool active = (s_last_rx_us != 0) && /* 曾经收到过音频包 */
                      (esp_timer_get_time() - s_last_rx_us < (int64_t)DISC_ACTIVE_HOLD_MS * 1000); /* 且在保持窗口内 */
        int period_ms = active ? DISC_ACTIVE_MS : DISC_IDLE_MS; /* 有流 20s / 无流 5s */
        if (period_ms != last_period_ms) { /* 周期变化时打一次日志, 便于现场验证 */
            last_period_ms = period_ms; /* 记录新周期 */
            ESP_LOGI(TAG, "discovery interval -> %d ms (%s audio stream)", period_ms, /* 日志: 当前周期与依据 */
                     active ? "receiving" : "no"); /* 有流 / 无流 */
        } /* 日志分支结束 */

        vTaskDelay(pdMS_TO_TICKS(period_ms)); /* 按当前周期等待下一轮广播 */
    } /* 主循环结束 */
} /* 函数结束 */

/* ============ BLE 连接提示音 (白噪声 + 呼吸式包络, 持续播放) ============ */
#define TONE_AMP       0.0625f                      /* 峰值幅度 -24dBFS (原 0.25 -12dBFS 降到 1/4, 四等分 -6dB) */
#define TONE_BREATH_S  4.0f                        /* 呼吸包络周期: 4s 一个完整起伏 (约 15 次/分) */
#define TONE_NOISE_SEED 0x12345678                 /* xorshift32 初始种子 (每次连接重置) */

static volatile bool s_tone_req = false; /* 提示音请求标志 (BLE 连接时置位, 断开时清除) */
static uint32_t      s_tone_pos = 0;     /* 包络位置计数 (仅 i2s 任务读写, 按呼吸周期回绕) */
static uint32_t      s_noise_st = TONE_NOISE_SEED; /* xorshift32 随机数状态 (仅 i2s 任务读写) */

/* xorshift32: 轻量伪随机, 每采样一次, 生成均匀分布 int32 白噪声 */
static inline int32_t noise_next(void) /* 返回 [-2^31, 2^31-1] 均匀分布样本 */
{ /* 函数体开始 */
    uint32_t x = s_noise_st; /* 取当前状态 */
    x ^= x << 13; /* 左移 13 位异或 */
    x ^= x >> 17; /* 右移 17 位异或 */
    x ^= x << 5;  /* 左移 5 位异或 */
    s_noise_st = x; /* 写回状态 */
    return (int32_t)x; /* 全 32bit 位随机 → 均匀白噪声 */
} /* 函数结束 */

/* 生成一包 (PAYLOAD_LEN) 的呼吸式白噪声写入 out; 由 BLE 连接状态控制启停 */
static void tone_fill(uint8_t *out) /* 填充一个包长度的白噪声样本 (32bit 小端) */
{ /* 函数体开始 */
    const float estep = 2.0f * (float)M_PI / (TONE_BREATH_S * (float)SAMPLE_RATE); /* 每采样包络相位步进 */
    const uint32_t breath_n = (uint32_t)(TONE_BREATH_S * (float)SAMPLE_RATE); /* 一个呼吸周期的采样数 */
    for (int i = 0; i + 3 < (int)PAYLOAD_LEN; i += 4) { /* 每 4 字节填一个 32bit 样本 (只填 payload 区, 不越界) */
        float env = 0.5f - 0.5f * cosf(estep * (float)s_tone_pos); /* 呼吸包络: 0 → 1 → 0 平滑起伏 */
        int32_t v = (int32_t)((float)noise_next() * TONE_AMP * env); /* 白噪声 × 包络 → int32 */
        out[i]     = (uint8_t)(v & 0xFF); /* bit[7:0] */
        out[i + 1] = (uint8_t)((v >> 8) & 0xFF); /* bit[15:8] */
        out[i + 2] = (uint8_t)((v >> 16) & 0xFF); /* bit[23:16] */
        out[i + 3] = (uint8_t)((v >> 24) & 0xFF); /* bit[31:24] */
        s_tone_pos++; /* 包络位置累加 */
    } /* 样本填充结束 */
    s_tone_pos %= breath_n; /* 按呼吸周期回绕, 避免浮点精度退化 */
} /* 函数结束 */

/* ============ ACM8815 延迟 Play ============
 * 芯片有时钟检测: Play 后 I2S 无持续数据会自动复位回 Digital Off.
 * init 只到 0x02 待机态, 首次有真实音频 (提示音/测试音/RTP 水位到达)
 * 时才进 Play; 幂等, 只执行一次 */
static bool s_amp_playing = false; /* ACM8815 是否已进入 Play */

static void amp_play_once(void) /* 首次真实音频前调用: 让 ACM8815 进 Play (此时 I2S 时钟已连续) */
{ /* 函数体开始 */
    if (s_amp_playing || !acm8815_present()) return; /* 已 Play 或功放不在线: 跳过 */
    if (acm8815_play() == ESP_OK) { /* 执行 Play Script (含 HPD/OCP workaround) */
        s_amp_playing = true; /* 标记已 Play, 后续不再重复 */
    } /* 成功分支结束 */
} /* 函数结束 */

/* ============ I2S 任务: ring -> DAC ============ */
static void i2s_task(void *arg) /* I2S 播放任务: 从环形缓冲取包写 I2S 输出, 基于 sender RTP ts 绝对墙钟定时 */
{ /* 任务入口 */
    (void)arg; /* 忽略入参 */

    uint8_t *out = heap_caps_malloc(s_pkt_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_DEFAULT); /* 分配单包输出缓冲区 (196B: 4B ts + 192B payload) */
    assert(out); /* 分配失败断言停机 */
    uint8_t *payload = out + sizeof(uint32_t); /* payload 指针: 跳过前 4B ts */

#if TEST_TONE_EN /* 测试音模式: 1kHz 正弦波直写 I2S, 与网络链路完全无关 */
    ESP_LOGW(TAG, "TEST TONE mode: 1kHz sine 80%%FS -> I2S (network bypassed)"); /* 日志: 已进入测试音模式 */
    const float step = 2.0f * (float)M_PI * 1000.0f / SAMPLE_RATE; /* 每采样点相位步进 (1kHz @48kHz) */
    float phase = 0.0f; /* 当前相位 */
    while (1) { /* 测试音主循环 */
        for (int i = 0; i + 3 < PAYLOAD_LEN; i += 4) { /* 每 4 字节填充一个 32bit 样本 */
            int32_t s = (int32_t)(sinf(phase) * 0.8f * 2147483647.0f); /* 80% 满幅防削波 (32bit 满幅) */
            phase += step; /* 相位累加 */
            if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI; /* 相位回绕到 [0,2pi) */
            payload[i]     = (uint8_t)(s & 0xFF); /* bit[7:0] 低字节 (小端) */
            payload[i + 1] = (uint8_t)((s >> 8) & 0xFF); /* bit[15:8] */
            payload[i + 2] = (uint8_t)((s >> 16) & 0xFF); /* bit[23:16] */
            payload[i + 3] = (uint8_t)((s >> 24) & 0xFF); /* bit[31:24] 高字节 (含符号) */
        } /* 样本填充循环结束 */
        i2s_write_lr(payload, PAYLOAD_LEN); /* 写测试音到 I2S (只写 192B payload) */
        amp_play_once(); /* 首包已写入, BCLK 连续 → 此时进 Play 安全 */
    } /* 测试音循环结束 */
#else /* 正常网络播放路径 */
    bool     started   = false;   /* 是否已越过水位线开始播放 */
    uint32_t anchor_ts = 0;       /* SNTP 未同步时的本地锚定: 首包 ts */
    int64_t  anchor_us = 0;       /* SNTP 未同步时的本地锚定: 首包写入时刻 */
    bool     use_wall  = false;   /* 是否用"发送端 ts = 墙钟"做绝对定时 (起播时判定, 对时成功后自动升级) */
    int64_t  sync_wait_us = 0;    /* 水位已够但 SNTP 未同步时的等待起点 (0=未开始计时) */
    bool     sync_timeout_logged = false; /* 对时超时降级只告警一次 */

    int64_t  next_tick_us = 0;    /* 播放节拍: 每轮至少 1ms, 防止 DMA 不阻塞时空转抢占 network */

    while (1) { /* 播放主循环 */
        /* 节拍器: 绝对时间对齐, 每轮推进 1ms.
         * 若 I2S 写入本身已阻塞约 1ms 则几乎不额外等待;
         * 若写入立即返回(DMA 未满)则补足, 避免循环高频空转抢占 network 任务
         * (优先级 5 > 4) 导致 UDP 邮箱溢出、rx 掉到 600/s. */
        {
            int64_t tnow = esp_timer_get_time();
            if (next_tick_us == 0 || tnow - next_tick_us > 20000) next_tick_us = tnow;
            next_tick_us += 1000;
            if (next_tick_us > tnow) {
                int64_t wus = next_tick_us - tnow;
                vTaskDelay(wus >= 1000 ? pdMS_TO_TICKS((uint32_t)(wus / 1000)) : 1);
            }
        }
        if (s_tone_req && ble_provision_is_connected()) { /* BLE 连接提示音 */
            tone_fill(payload);
            i2s_write_lr(payload, PAYLOAD_LEN); /* 阻塞 ≈1ms, 天然 yield */
            amp_play_once();
            continue;
        }
        if (s_tone_req && !ble_provision_is_connected()) { s_tone_req = false; }

        if (!started) { /* 等水位 + 等对时 (方案 A §F1-③ / §F2-4) */
            if (ring_pkt_cnt() < TARGET_WMARK) { /* 水位不够 */
                sync_wait_us = 0; /* 没攒够包时不计对时等待 */
                vTaskDelay(1); /* yield 给网络任务/IDLE, 不写静音 (避免 DMA 排队延迟) */
                continue;
            }
            /* 水位已够但 SNTP 还没同步: 先缓存不出声, 避免各台按各自上电时刻起播导致台间错位.
             * 最多等 SYNC_WAIT_MS, 超时降级为本地锚定播放 (能出声但台间可能不同步) */
            if (!time_is_synced()) {
                if (sync_wait_us == 0) sync_wait_us = esp_timer_get_time(); /* 开始计时 */
                if (esp_timer_get_time() - sync_wait_us < (int64_t)SYNC_WAIT_MS * 1000) {
                    vTaskDelay(1); /* 继续等对时, ring 继续积累 (上限由 RING_HIGH_PKTS 裁剪) */
                    continue;
                }
                if (!sync_timeout_logged) { /* 只告警一次 */
                    sync_timeout_logged = true;
                    ESP_LOGW(TAG, "SNTP 未在 %dms 内同步 (server=%s), 降级为本地锚定播放 (台间可能不同步)", /* 日志 */
                             SYNC_WAIT_MS, s_sntp_server);
                }
            }
            ring_pop(out); /* 先出队首包 (用于锚定) */
            anchor_ts = *(uint32_t *)out;
            anchor_us = esp_timer_get_time();
            use_wall  = ts_looks_wall(anchor_ts); /* 起播判定: 已对时且 ts 与墙钟一致才用绝对定时 */
            started   = true;
            s_dbg_use_wall = use_wall; /* 供 stat 行打印当前定时模式 */
            i2s_write_lr(payload, PAYLOAD_LEN); /* 首包立即播, DMA 管线从此跑起来 */
            amp_play_once();
            ESP_LOGI(TAG, "playback start: first_ts=%lu mode=%s (DMA=%dms, lead=%dms, off=%ldms)", /* 日志: 起播模式与时间基准差 */
                     (unsigned long)anchor_ts, use_wall ? "wall-clock" : "local-anchor", /* 首包 ts / 定时模式 */
                     I2S_DMA_DEPTH_MS, PLAYOUT_LEAD_MS, (long)s_ntp_offset_ms); /* DMA 深度 / 播放前瞻 / 墙钟-发端 ts */
            continue;
        }

        /* ===== 稳态: 绝对墙钟定时播出 ===== */
        /* 积压上限: ring 长期满 = 延迟锁死 (后续真实包全被 ring_push 丢掉).
         * 超过 RING_HIGH_PKTS 就丢最旧包追赶到 RING_TRIM_PKTS, 把延迟拉回设计值. */
        {
            uint32_t n = ring_pkt_cnt(); /* 当前积压包数 */
            if (n > (uint32_t)RING_HIGH_PKTS) { /* 积压过高 */
                uint32_t drop = n - (uint32_t)RING_TRIM_PKTS; /* 需要丢弃的包数 */
                while (drop--) { /* 丢弃最旧包 */
                    if (!ring_pop(out)) break; /* 队列已空则停止 */
                    s_stats.missed++; /* 计入缺包 (这些包不会被播出) */
                }
            }
        }

        if (!ring_pop(out)) { /* ring 空 → 欠载 */
            s_stats.underrun++;
            i2s_write_lr(s_silence + sizeof(uint32_t), PAYLOAD_LEN); /* 写静音保持节拍(阻塞) */
            continue;
        }

        uint32_t ts = *(uint32_t *)out; /* sender 的 RTP ts (48kHz 时钟) */

        if (ts == 0) { /* ts=0 的填充包 (非 drain 补的静音) → 直接播, 不参与定时 */
            i2s_write_lr(payload, PAYLOAD_LEN);
            continue;
        }

        /* 对时成功后自动升级: 起播时若 SNTP 未同步会先按本地锚定跑, 同步完成后切到绝对墙钟,
         * 否则多台音箱会一直保持各自锚定的相位差 (方案 A §F1-③) */
        if (!use_wall && time_is_synced() && ts_looks_wall(ts)) {
            use_wall = true; /* 切绝对定时 */
            s_dbg_use_wall = true; /* stat 行同步显示 */
            s_dbg_abs_off_ms = 0; /* 新时基清零纠偏 */
            ESP_LOGI(TAG, "timing mode -> wall-clock (SNTP synced, off=%ldms, ts=%lu)", /* 日志: 模式切换 */
                     (long)s_ntp_offset_ms, (unsigned long)ts);
        }

        /* late_ms: 该包相对"应写入 DMA 的时刻"的偏移 (ms); >0 落后, <0 超前 */
        int32_t late_ms;
        if (use_wall) { /* 发送端 ts 是墙钟且 SNTP 已同步: 多设备同一时刻播同一帧 */
            int32_t diff = (int32_t)(wall_rtp_now() - ts); /* 32bit 有符号差值, 自动处理回绕 */
            late_ms = diff / 48 + I2S_DMA_DEPTH_MS - PLAYOUT_LEAD_MS + s_dbg_abs_off_ms;
        } else { /* 发送端 ts 非墙钟(实测是开机采样计数): 本地锚定相对播放, 保证本机连续出声 */
            int64_t now_ms = (esp_timer_get_time() - anchor_us) / 1000;
            int64_t due_ms = (int64_t)((int32_t)(ts - anchor_ts)) / 48;
            late_ms = (int32_t)(now_ms - due_ms) + I2S_DMA_DEPTH_MS - PLAYOUT_LEAD_MS;
        }
        s_dbg_late_ms = late_ms;   /* 供 stat 日志诊断 */
        if (late_ms < s_dbg_late_min) s_dbg_late_min = late_ms; /* 统计窗内最小 late (超前程度) */
        if (late_ms > s_dbg_late_max) s_dbg_late_max = late_ms; /* 统计窗内最大 late (落后程度) */

        if (late_ms < -500) { /* 时基跳变: sender 重启/换曲/大段跳包后 ts 起点变化 → 重锚定 */
            s_stats.late_jump++; /* 统计: 时基跳变次数 (发端 ts 基准突变) */
            if (use_wall) {
                s_dbg_abs_off_ms += -late_ms; /* 绝对模式: 一次性抵消跳变 */
            } else {
                anchor_ts = ts;
                anchor_us = esp_timer_get_time();
            }
            ESP_LOGW(TAG, "timebase jumped (ts=%lu late=%ldms mode=%s), re-anchored", /* 日志: 发端时间戳基准跳变 */
                     (unsigned long)ts, (long)late_ms, use_wall ? "wall" : "anchor");
            continue;
        }

        if (late_ms > LATE_DROP_MS) { /* 落后超过缓冲垫: 丢掉这个包, 不能把延迟永久推后 */
            s_stats.missed++;
            s_stats.late_drop++; /* 统计: 因时间戳落后被丢弃的帧 */
            /* 限速告警 (每 500ms 最多一条): 明确指出"收到包但时间不一致被丢", 用于区分无声原因 */
            static int64_t s_last_late_log_us;
            int64_t now_us = esp_timer_get_time();
            if (now_us - s_last_late_log_us > 500000) {
                s_last_late_log_us = now_us;
                ESP_LOGW(TAG, "late drop: ts=%lu late=%ldms mode=%s off=%ldms ring=%lu (audio ts mismatch)", /* 日志 */
                         (unsigned long)ts, (long)late_ms, use_wall ? "wall" : "anchor", /* 包 ts / 落后量 / 模式 */
                         (long)s_ntp_offset_ms, (unsigned long)ring_pkt_cnt()); /* 墙钟-发端 ts / 积压 */
            }
            /* 本地锚定模式下, 发端 ts 速率与本机时钟存在微小偏差 (实测 ~0.02%),
             * 累积后 late 会稳定卡在 LATE_DROP_MS 之上且永不回落 → 持续丢包直到无声.
             * 因此 anchor 模式把重锚定阈值降到 100ms (网络抖动实测 ≤44ms, 不会误触发);
             * wall 模式仍用 250ms, 避免与墙钟纠偏互相打架 */
            if (late_ms > (use_wall ? 250 : 100)) { /* 严重滞后(时基错/发送端偏快): 清积压后用最新包重建时基 */
                s_stats.late_fix++; /* 统计: 强制重建次数 */
                int guard = 0;
                while (ring_pkt_cnt() > 1 && guard++ < (int)RING_PKTS) {
                    if (!ring_pop(out)) break;
                }
                if (ring_pop(out)) { /* 取最新包 */
                    uint32_t nts = *(uint32_t *)out;
                    if (nts != 0) {
                        if (use_wall) {
                            s_dbg_abs_off_ms -= late_ms; /* 绝对模式: 把累积偏差一次性抵消 */
                        } else {
                            anchor_ts = nts;         /* 本地模式: 直接重锚定 */
                            anchor_us = esp_timer_get_time();
                        }
                    }
                    i2s_write_lr(payload, PAYLOAD_LEN); /* 播最新包, 立即恢复输出 */
                } else {
                    i2s_write_lr(s_silence + sizeof(uint32_t), PAYLOAD_LEN);
                }
            }
            continue;
        }

        /* 超前(late<0)直接播, 不等待: I2S DMA 写满自然阻塞 ≈1ms, 它本身就是 1 包/ms 节拍器.
         * 缓冲垫由起播水位(24 包)建立, 不靠在这里等待 (等待会饿死消费端). */

        /* 注: 缺口静音由 drain_pkts() 在入队时统一补齐(且不会把延迟推后),
         * 这里不再按 ts 差二次补静音, 否则大跳包后会重复补几十 ms, 延迟累积. */

        /* 正常: 播当前包 (阻塞 ≈1ms, 天然 yield) */
        i2s_write_lr(payload, PAYLOAD_LEN);
    } /* 播放循环结束 */
#endif /* TEST_TONE_EN 分支结束 */
} /* 函数结束 */

/* ---------- BLE 连接/配网回调 (状态灯) ---------- */

static void on_ble_conn(bool connected) /* BLE 连接/断开回调 (由 ble_provision 调用) */
{ /* 函数体开始 */
    if (connected) { /* BLE 连接成功: 开启呼吸式白噪声提示音 */
        s_tone_pos = 0; /* 包络从头开始 (计数清零) */
        s_noise_st = TONE_NOISE_SEED; /* 噪声序列重置 */
        s_tone_req = true; /* 置位请求, i2s 任务循环播放直到断开 */
        ESP_LOGI(TAG, "BLE connected -> start white noise (breathing %.1fs period)", TONE_BREATH_S); /* 日志 */
    } else { /* BLE 断开: 立即停止提示音 */
        s_tone_req = false; /* 清除请求, i2s 任务下一循环即停 */
        ESP_LOGI(TAG, "BLE disconnected -> stop tone"); /* 日志 */
    } /* 断开分支结束 */

    /* WiFi 重连黄灯优先级最高: 正在黄灯闪时 BLE 回调不覆盖, 避免与 WiFi 故障指示灯竞争 */
    if (s_led_state == LED_ST_WIFI_RECONNECT) {
        ESP_LOGI(TAG, "BLE %s (ignore: WiFi reconnecting, yellow LED takes priority)", connected ? "connected" : "disconnected");
        return; /* 让黄灯继续闪 */
    } /* WiFi 重连优先分支结束 */
    s_led_state = connected ? LED_ST_BLUE_BLINK : s_led_base; /* 连接→蓝闪1Hz / 断开→回基准状态 */
    ESP_LOGI(TAG, "BLE %s -> LED %s", connected ? "connected" : "disconnected", /* 日志 */
             connected ? "blue blink 1Hz" : (s_led_base == LED_ST_BG_BLINK ? "blue/green blink" : "solid green")); /* 日志参数 */
} /* 函数结束 */

/* ============ 配网"停止发送"通告: 直接在旧 WiFi 上广播 tx_enable=0, 再重启连新 WiFi ============ */

/* 独立任务执行通告 (配网回调跑在 NimBLE host 任务里, 不能长阻塞):
 * 工作模式下 BLE 与 WiFi 共存, 收到新配置时旧 WiFi 仍处于连接状态,
 * 无需重连: 直接读当前 IP 连发 3 个 tx_enable=0 的发现包 (告知发送端停止向本机旧 IP 发送),
 * 随后由 ble_provision 延时重启, 重启后工作模式才用【新配置】连新 WiFi。
 * 纯配网模式 (无旧 WiFi 连接) 无流可停, 直接跳过通告 */
static void stop_announce_task(void *arg) /* 配网停止通告任务 */
{ /* 任务入口 */
    (void)arg; /* 忽略入参 */

    prov_config_t cfg; /* 旧配置快照 (新配置保存前由 ble_provision 捕获) */
    if (!ble_provision_get_prev_config(&cfg)) { /* 无旧配置 (首次配网) */
        ESP_LOGI(TAG, "stop announce: no prev config (first provision), skip"); /* 日志: 跳过通告 */
        vTaskDelete(NULL); /* 结束任务 (重启由 ble_provision 延时任务负责) */
        return; /* 不再继续 */
    } /* 无旧配置分支结束 */

    /* 直接读当前 (旧 WiFi) IP: 工作模式下 WiFi 已连接; 纯配网模式 netif 不存在或 IP 为 0 */
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"); /* STA netif */
    esp_netif_ip_info_t ipi = { 0 }; /* IP 信息 */
    if (netif == NULL || esp_netif_get_ip_info(netif, &ipi) != ESP_OK || ipi.ip.addr == 0) { /* 当前无连接 */
        ESP_LOGI(TAG, "stop announce: no active WiFi connection, skip"); /* 日志: 无流可停 */
        vTaskDelete(NULL); /* 结束任务 */
        return; /* 不再继续 */
    } /* 无连接分支结束 */

    /* 在旧 WiFi 上连发 3 个 tx_enable=0 的"停止发送"通告包 (旧 ID/旧声道/旧 IP) */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); /* UDP socket */
    if (sock >= 0) { /* 创建成功 */
        int bc = 1; /* 广播使能 */
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc, sizeof(bc)); /* 允许广播 */
        struct sockaddr_in dst = { 0 }; /* 广播目标 */
        dst.sin_family = AF_INET; /* IPv4 */
        dst.sin_port = htons(DISC_PORT); /* 6000 */
        dst.sin_addr.s_addr = ipi.ip.addr | ~ipi.netmask.addr; /* 同网段定向广播 */
        for (uint16_t i = 0; i < DISC_BURST_N; i++) { /* 连发 3 包 */
            disc_send(sock, &dst, cfg.device_id, (uint8_t)(1u << (cfg.slot_index & 7)), 0, ipi.ip.addr, i); /* tx_enable=0 */
            if (i + 1 < DISC_BURST_N) vTaskDelay(pdMS_TO_TICKS(DISC_BURST_GAP_MS)); /* 包间隔 */
        } /* 连发结束 */
        close(sock); /* 关闭 socket */
        ESP_LOGI(TAG, "stop announce sent on current WiFi (tx_enable=0, id=%u slot=0x%02X ip=%u.%u.%u.%u), rebooting to new config", /* 日志 */
                 cfg.device_id, (unsigned)(1u << (cfg.slot_index & 7)), /* 旧设备编号与声道 bit */
                 (unsigned)ipi.ip.addr & 0xFF, (unsigned)(ipi.ip.addr >> 8) & 0xFF, /* 旧 IP 前两字节 */
                 (unsigned)(ipi.ip.addr >> 16) & 0xFF, (unsigned)(ipi.ip.addr >> 24) & 0xFF); /* 旧 IP 后两字节 */
    } /* 发送分支结束 */
    vTaskDelete(NULL); /* 结束任务: 随后 ble_provision 延时重启, 工作模式用新配置连 WiFi */
} /* 函数结束 */

static void on_prov_done(void) /* 配网成功回调 (由 ble_provision 调用) */
{ /* 函数体开始 */
    s_led_base  = LED_ST_SOLID_GREEN; /* 基准状态转常绿 */
    s_led_state = LED_ST_SOLID_GREEN; /* 配网完成: 常绿 */
    s_tx_enable = 0; /* 新配置生效前先置 0: 通告发送端停止向本机发送 (重启后恒为 1) */
    xTaskCreate(stop_announce_task, "stop_ann", 4096, NULL, 3, NULL); /* 独立任务通告, 不阻塞 BLE host */
    ESP_LOGI(TAG, "provision done -> announce tx_enable=0 then reboot (LED solid green)"); /* 日志 */
} /* 函数结束 */

/* ---------- WiFi/IP 事件回调: 掉线/重连驱动黄灯闪 ---------- */
static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base, /* 事件基 */
                                   int32_t event_id, void *event_data) /* 事件 ID + 数据 */
{ /* 函数体开始 */
    (void)arg; /* 忽略用户参数 */
    if (event_base == WIFI_EVENT) { /* WiFi 事件 */
        switch (event_id) { /* 按事件类型处理 */
        case WIFI_EVENT_STA_DISCONNECTED: { /* STA 已从 AP 断开 */
            wifi_event_sta_disconnected_t *dis = (wifi_event_sta_disconnected_t *)event_data; /* 断连详情 */
            s_wifi_disc_cnt++; /* 累计断连次数 (link 诊断) */
            s_wifi_last_reason = dis->reason; /* 记录原因码, 供 link 行显示 */
            ESP_LOGW(TAG, "WiFi STA disconnected reason=%d, reconnecting...", dis->reason); /* 日志: 断开原因 */
            s_led_state = LED_ST_WIFI_RECONNECT; /* 切黄灯闪 (重连优先级最高) */
            esp_wifi_connect(); /* 显式调用 connect 立即启动重连, 避免等待内部退避 */
            break; /* 分支结束 */
        } /* DISCONNECTED 分支结束 */
        case WIFI_EVENT_STA_CONNECTED: /* 802.11 关联成功 (尚未 DHCP) */
            ESP_LOGI(TAG, "WiFi STA associated, waiting for IP...");
            break; /* 分支结束 */
        default: /* 其它 WiFi 事件忽略 */
            break; /* 分支结束 */
        } /* WiFi 事件 switch 结束 */
    } else if (event_base == IP_EVENT) { /* IP 事件 */
        switch (event_id) { /* 按事件类型处理 */
        case IP_EVENT_STA_GOT_IP: { /* DHCP 成功拿到 IP → WiFi 恢复正常 */
            ip_event_got_ip_t *got = (ip_event_got_ip_t *)event_data; /* IP 详情 */
            ESP_LOGI(TAG, "WiFi got IP " IPSTR ", link up!", IP2STR(&got->ip_info.ip)); /* 日志: 网络就绪 */
            ble_provision_set_ip(got->ip_info.ip.addr); /* BLE 广播厂商数据同步本机 IP (扫描即可读到) */
            /* 恢复正常灯态: 如果 BLE 已连上则蓝闪, 否则常绿 (但 WiFi 重连黄灯必须清掉) */
            /* 注意: 这里不知道 BLE 当前状态, 最简单的做法是先回基准状态,
             * 已连接 BLE 时 on_ble_conn 之前已经切过了; 若 BLE 状态不确定,
             * 直接设常绿让后续 BLE 事件驱动最干净 */
            s_led_state = s_led_base; /* 回基准状态 (工作=常绿 / 配网=蓝绿交替) */
            break; /* 分支结束 */
        } /* GOT_IP 分支结束 */
        case IP_EVENT_STA_LOST_IP: /* DHCP 租约丢失 */
            ESP_LOGW(TAG, "WiFi lost IP"); /* 日志 */
            ble_provision_set_ip(0); /* 广播里的 IP 清零 */
            s_led_state = LED_ST_WIFI_RECONNECT; /* 切黄灯闪 */
            break; /* 分支结束 */
        default: /* 其它 IP 事件忽略 */
            break; /* 分支结束 */
        } /* IP 事件 switch 结束 */
    } /* 事件基分支结束 */
} /* 事件回调函数结束 */

static void led_task(void *arg) /* LED 状态灯任务: 按 s_led_state 驱动 WS2812B */
{ /* 任务入口 */
    (void)arg; /* 忽略入参 */
    bool phase = true; /* 闪烁相位: 蓝绿态 true=蓝/false=绿; 蓝闪态 true=亮/false=灭 */

    /* 任务启动时先强制熄灭一次: 保证 "初始化后先灭灯, 再按状态亮灯" 的视觉顺序,
     * 即便 ws2812_init 末尾已发过全黑, 这里再发一次仍是 O(1) 开销 */
    ws2812_set(0, 0, 0); /* 启动瞬间确保熄灭 */

    while (1) { /* 主循环 */
        switch (s_led_state) { /* 按状态输出 */
        case LED_ST_BG_BLINK: /* 蓝绿交替闪 (配网等待连接) */
            ws2812_set(0, phase ? 0 : 255, phase ? 255 : 0); /* 蓝 / 绿 */
            phase = !phase; /* 切换相位 */
            vTaskDelay(pdMS_TO_TICKS(LED_BG_HALF_MS)); /* 250ms */
            break; /* 分支结束 */
        case LED_ST_BLUE_BLINK: /* 蓝色 1Hz 闪烁 (BLE 已连接) */
            ws2812_set(0, 0, phase ? 255 : 0); /* 蓝亮 / 灭 */
            phase = !phase; /* 切换相位 */
            vTaskDelay(pdMS_TO_TICKS(LED_BLUE_HALF_MS)); /* 500ms → 周期 1s */
            break; /* 分支结束 */
        case LED_ST_WIFI_RECONNECT: /* 黄灯 1Hz 闪烁 (WiFi 未连上 / 中途掉线, 正在重连) */
            ws2812_set(phase ? 255 : 0, phase ? 255 : 0, 0); /* 黄亮 / 灭 (R+G=黄) */
            phase = !phase; /* 切换相位 */
            vTaskDelay(pdMS_TO_TICKS(LED_YELLOW_HALF_MS)); /* 500ms → 周期 1s */
            break; /* 分支结束 */
        case LED_ST_SOLID_GREEN: /* 常绿 (已配网) */
        default: /* 默认同上 */
            ws2812_set(0, 10, 0); /* 常绿 */
            vTaskDelay(pdMS_TO_TICKS(200)); /* 低频保持 (灯珠自锁存, 仅防状态切换延迟) */
            break; /* 分支结束 */
        } /* switch 结束 */
    } /* 主循环结束 */
} /* 函数结束 */

/* ============ BOOT 键: 长按 3 秒清空配置并重启进入配网模式 ============ */
static void boot_key_task(void *arg) /* BOOT 键长按检测任务: 长按 3 秒清除配网并重启 */
{ /* 任务入口 */
    gpio_config_t io = { /* GPIO 配置结构体初始化 */
        .pin_bit_mask = 1ULL << BOOT_KEY_GPIO, /* 选择 BOOT 键所在 GPIO (9) */
        .mode         = GPIO_MODE_INPUT, /* 配置为输入模式 */
        .pull_up_en   = GPIO_PULLUP_ENABLE,   /* BOOT 键按下拉低 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /* 禁用下拉 (BOOT 键默认上拉) */
        .intr_type    = GPIO_INTR_DISABLE, /* 不使用中断 */
    }; /* GPIO 配置结构体结束 */
    gpio_config(&io); /* 应用 GPIO 配置 */

    uint32_t press_ms = 0; /* 按键按下累计时长 (毫秒) */
    while (1) { /* 轮询循环 */
        if (gpio_get_level(BOOT_KEY_GPIO) == 0) { /* 检测到按键按下 (低电平) */
            press_ms += 50; /* 累计按压时长 50ms */
            if (press_ms >= BOOT_KEY_HOLD_MS) { /* 累计达到长按阈值 */
                ESP_LOGW(TAG, "BOOT held %u ms, clearing config and restarting into provisioning...", /* 日志提示清除配置并进入配网 */
                         BOOT_KEY_HOLD_MS); /* 打印阈值参数 */
                prov_config_clear(); /* 清除 NVS 中的配网配置 */
                esp_restart(); /* 重启系统进入配网流程 */
            } /* 长按分支结束 */
        } else { /* 按键未按下 */
            press_ms = 0; /* 清零累计时长 */
        } /* 未按下分支结束 */
        vTaskDelay(pdMS_TO_TICKS(50)); /* 每 50ms 轮询一次 */
    } /* 轮询循环结束 */
    (void)arg; /* 忽略入参 (不可达, 防警告) */
} /* 函数结束 */

/* ============ I2S 输出初始化 ============ */
static void i2s_out_init(bool full, i2s_chan_handle_t *tx) /* I2S 输出初始化: full=8 声道 TDM / single=单声道 STD */
{ /* 函数体开始 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER); /* I2S0 主模式通道配置 */
    /* 欠载自动补零: 无数据可发时 DMA 输出 0, 而不是反复回放上一帧 DMA 内容
     * (默认 false 会把最后的提示音帧无限循环, 表现为"断开蓝牙后一直响 1kHz") */
    chan_cfg.auto_clear = true; /* 无数据时自动发零 */
    /* DMA 深度必须显式设置: I2S_CHANNEL_DEFAULT_CONFIG 默认 6 描述符 x 240 帧,
     * 48kHz / 32bit stereo 下 = 6 x 5ms = 30ms 缓冲, 直接把 <30ms 的延迟预算吃光.
     * 这里 4 x 96 帧 = 384 帧 = 8ms, 与 I2S_DMA_DEPTH_MS 的纠偏窗口一致 */
    chan_cfg.dma_desc_num  = 4; /* DMA 描述符个数 */
    chan_cfg.dma_frame_num = 96; /* 每个描述符 96 帧 (96 x 8B = 768B = 2ms) */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, tx, NULL)); /* 创建 TX 通道 (无 RX) */

    if (full) { /* full 模式: TDM 8 slot */
        /* full: TDM 8 slot 输出, 48kHz/32bit, slot0..7 = L/R/C/LFE/SL/SR/SBL/SBR */
        i2s_tdm_config_t tdm_cfg = { /* TDM 配置结构体初始化 */
            .clk_cfg  = I2S_TDM_CLK_DEFAULT_CONFIG(SAMPLE_RATE), /* 时钟配置: 48kHz 采样 */
            .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO, /* slot 配置: 32bit, 立体声槽模式 */
                                                           I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | /* 使能 slot0-2 (前 3 个声道) */
                                                           I2S_TDM_SLOT3 | I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | /* 使能 slot3-5 (中 3 个声道) */
                                                           I2S_TDM_SLOT6 | I2S_TDM_SLOT7), /* 使能 slot6-7 (后 2 个声道) */
            .gpio_cfg = { /* GPIO 配置开始 */
                .mclk = I2S_GPIO_MCLK, /* MCLK 主时钟引脚 */
                .bclk = I2S_GPIO_BCLK, /* BCLK 位时钟引脚 */
                .ws   = I2S_GPIO_WS, /* WS 帧同步引脚 */
                .dout = I2S_GPIO_DOUT, /* DOUT 数据输出引脚 */
                .din  = GPIO_NUM_NC, /* DIN 未使用 */
            }, /* GPIO 配置结束 */
        }; /* TDM 配置结构体结束 */
        /* 48kHz 时钟树: BCLK = 8slot x32bit x48k = 12.288MHz, MCLK = 384fs = 18.432MHz */
        tdm_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; /* MCLK 倍频设为 384; 24-bit 下必须是 3 的倍数, 256 非法 */
        tdm_cfg.slot_cfg.slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | /* 显式设置 8 声道 slot 掩码 */
                                     I2S_TDM_SLOT3 | I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | /* slot3-5 掩码位 */
                                     I2S_TDM_SLOT6 | I2S_TDM_SLOT7; /* slot6-7 掩码位 */
        ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(*tx, &tdm_cfg)); /* 初始化 I2S 通道为 TDM 模式 */
        ESP_LOGI(TAG, "I2S0 TDM tx: %dHz x%dch x%dbits", SAMPLE_RATE, CH_NUM, BIT_DEPTH); /* 日志: TDM 输出参数 */
    } else { /* full 分支结束 */
        /* single: I2S 标准输出, 48kHz/32bit, 双槽 (L/R 写入同一份数据)
         * ⚠️ 不能用 I2S_SLOT_MODE_MONO: ESP-IDF 的 MONO 只是把 active_slot 记成 1
         * (仅影响 DMA 容量计算), 并不在硬件上把数据复制到另一个槽. 物理上仍是
         * 64fs 双槽帧, 若只写单声道数据, 每帧 8 字节会被拆成 L/R 两个不同样本,
         * 采样率错位且另一槽无数据 → 功放侧收不到正确 PCM (只有底噪). */
        i2s_std_config_t std_cfg = { /* single 模式: STD 单槽 */
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_MCLK,
                .bclk = I2S_GPIO_BCLK,
                .ws   = I2S_GPIO_WS,
                .dout = I2S_GPIO_DOUT,
                .din  = GPIO_NUM_NC,
            },
        };
        /* single 模式时钟: BCLK = 2slot x32bit x48k = 3.072MHz = 64fs (32bit 槽);
         * ACM8815 内部 PLL 以 BCLK 为参考 (无需 MCLK), 经 I2C 配置 0x07=0x03 (I2S 32bit)
         * 与之匹配; 芯片上电默认 24bit, 必须显式配置 32bit */
        std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384; /* MCLK = 384fs (48kHz*384=18.432MHz), 满足 >192fs 要求 */
        /* 槽位宽必须显式 32bit: I2S_SLOT_BIT_WIDTH_AUTO 可能落到 16bit,
         * 那样 BCLK=48k*2*16=1.536MHz, 每帧只有 4B, 写入 384B 实际只覆盖 2ms,
         * DMA 每 2ms 才腾出空间 → i2s 每秒只消费约 500 包(实际采样率变 24kHz),
         * 表现为 ring 长期积压(延迟大) + 大量丢包/补静音(叭叭声) */
        std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx, &std_cfg)); /* 初始化 I2S 通道为标准模式 */
        ESP_LOGI(TAG, "I2S0 STD tx: %dHz stereo(L=R) x%dbits, BCLK=64fs", SAMPLE_RATE, BIT_DEPTH); /* 日志: STD 输出参数 */
    } /* single 分支结束 */

    ESP_ERROR_CHECK(i2s_channel_enable(*tx)); /* 使能 I2S 通道开始输出 */
} /* 函数结束 */

/* ============ 输出链路初始化 (I2S + ACM8815): 配网/工作模式共用 ============ */
static void audio_out_init(bool full, uint16_t slot) /* 初始化 I2S 输出与 DAC 配置 */
{ /* 函数体开始 */
    if (full) { /* 单声道流不含 8ch TDM 数据, full 模式无法出声 */
        ESP_LOGW(TAG, "output_mode=full not supported with mono stream, fallback to single"); /* 告警: 回退单声道 */
        full = false; /* 强制回退 single */
    } /* full 回退分支结束 */
    s_full       = full; /* 保存输出模式 (单声道流恒为 single) */
    s_slot_index = slot; /* 保存本机声道序号 (用于 BLE/WiFi 命名与发现广播) */
    s_pkt_bytes  = sizeof(uint32_t) + PAYLOAD_LEN; /* 单包字节数 = 4B ts + 192B payload = 196B */

    memset(s_silence, 0, sizeof(s_silence)); /* 清零静音缓冲 (ts=0, payload=全零) */

    /* I2S 输出: C5 为功放提供 BCLK/WS/数据, ACM8815 从模式 (PLL 以 BCLK 为参考) */
    i2s_out_init(full, &s_tx); /* 初始化 I2S 输出通道 (含 enable, 开始输出 BCLK/LRCK) */

    /* ====== 灌一包静音到 I2S: 预填 DMA, 让 BCLK/LRCK 尽快连续输出 ======
     * ACM8815 内部 DSP 会检测 I2S 时钟有效性 —— Play 后 BCLK/LRCK 不稳定,
     * 芯片会自动从 Play 回退到 Digital Off, 因此 Play 已延后到 amp_play_once()
     * (首个真实音频时). 这里预灌静音只是让时钟链路尽早建立.
     * 只写 PAYLOAD_LEN (192B) payload, 跳过 s_silence 前 4B ts 字段 */
    i2s_write_lr(s_silence + sizeof(uint32_t), PAYLOAD_LEN); /* mono 192B → STEREO 自动扩展 L/R = 384B */
    ESP_LOGI(TAG, "I2S seeded with %d bytes silence before ACM8815 Play Script", (int)(sizeof(s_silence) * 2));

    /* ACM8815 配置: PDN=GPIO18 拉高上电 -> MUTE=GPIO19 拉低解除静音 ->
     * I2C 复位 -> 32bit I2S -> 默认音量 20% -> Play;
     * 注意与 ES9018K2M 不同: 芯片上电默认 Digital Off (无声), I2C 初始化失败 = 无声,
     * 没有"上电默认出声"的回退, 失败时重点排查 I2C 接线;
     * PDN/MUTE 两路硬件 GPIO 独立控制, 掉电需重走 init 流程 */
    esp_err_t dac_err = acm8815_init(); /* 初始化数字功放 */
    if (dac_err != ESP_OK) { /* 功放配置失败 */
        ESP_LOGE(TAG, "ACM8815 I2C init failed (%s), device stays Digital-Off (NO SOUND!)", /* 严重告警: 无声 */
                 esp_err_to_name(dac_err)); /* 打印错误名 */
    } /* 失败分支结束 */
} /* 函数结束 */

/* ============ 主入口 ============ */
void app_main(void) /* 程序入口: 加载配置, 无配置则进 BLE 配网, 否则启动音频链路 */
{ /* 函数体开始 */
    /* 全局初始化 (仅一次): NVS / netif / event loop */
    setenv("TZ", "CST-8", 1); /* 时区设为北京时间 (UTC+8, 无夏令时); 否则 localtime 输出 UTC */
    tzset(); /* 使 TZ 立即生效, 后续日志/stat 的时间都是本地时间 */
    ESP_ERROR_CHECK(nvs_flash_init()); /* 初始化 NVS flash (配置存储) */
    ESP_ERROR_CHECK(esp_netif_init()); /* 初始化网络接口 */
    ESP_ERROR_CHECK(tcp_log_init(0));  /* TCP 远程日志: telnet/nc <ip> 3333 (不影响 USB 串口) */
    ESP_ERROR_CHECK(esp_event_loop_create_default()); /* 创建默认事件循环 */

    /* BOOT 键监听: 任意模式下长按 3 秒均可重新配网 (单核, Core 0) */
    xTaskCreatePinnedToCore(boot_key_task, "boot_key", 2048, NULL, 2, NULL, 0); /* 创建 BOOT 键长按检测任务 (核0, 优先级2) */

    /* 读取配网配置, 决定工作模式 */
    bool has_cfg = (prov_config_load(&s_cfg) == ESP_OK && prov_config_valid(&s_cfg)); /* 加载并校验 NVS 中的配网配置 */
    if (!has_cfg) { /* 无有效配置 */
        /* 模式0: BLE 配网 (关 WiFi 开 BLE), 配网成功后自动重启, 此函数不返回 */
        ESP_LOGW(TAG, "no valid config, entering BLE provisioning..."); /* 日志提示进入 BLE 配网 */
        /* 配网模式也初始化输出链路 (I2S + ACM8815), 否则 BLE 连接提示音无法出声;
         * slot 固定 0 (配网阶段尚无声道配置), 环形缓冲仅给 i2s 任务提供空环 */
        audio_out_init(false, 0); /* 初始化 I2S + DAC (单声道) */
        ring_init(RING_PKTS * s_pkt_bytes, s_pkt_bytes); /* 空环形缓冲 (无网络生产者) */
        xTaskCreatePinnedToCore(i2s_task, "i2s", 4096, NULL, 5, &s_h_i2s, 0); /* I2S 任务: 播放 BLE 提示音 (核0, 优先级5) */
        ws2812_init(LED_GPIO_WS2812); /* 初始化状态灯 (WS2812B) */
        s_led_base  = LED_ST_BG_BLINK; /* 配网基准状态: 蓝绿交替 */
        s_led_state = LED_ST_BG_BLINK; /* 配网中(未连接): 蓝绿交替闪烁 */
        ble_provision_set_conn_cb(on_ble_conn); /* 注册 BLE 连接/断开回调 (切状态灯) */
        ble_provision_set_done_cb(on_prov_done); /* 注册配网成功回调 (状态灯转常绿) */
        xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 1, NULL, 0); /* 状态灯任务 (核0, 优先级1) */
        ble_provision_start(); /* 启动 BLE 配网服务 (阻塞式) */
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));   /* 不应到达 */
    } /* 无配置分支结束 */

    ESP_LOGW(TAG, "existing valid config detected, entering work mode (BLE off); hold BOOT 3s to clear config and re-enter provisioning"); /* 日志: 提示已有配置, 长按BOOT可清配置重进配网 */
    ESP_LOGI(TAG, "config: mode=%s ssid=%s slot=%u id=%u out=%s port=%u (unicast)", /* 日志: 打印当前配置摘要 */
             s_cfg.mode, s_cfg.ssid, s_cfg.slot_index, s_cfg.device_id, /* 配置参数: 模式/SSID/slot/设备编号 */
             s_cfg.output_mode[0] ? s_cfg.output_mode : "single", /* 配置参数: 输出模式 (默认 single) */
             (unsigned)DEF_AUDIO_PORT); /* 音频接收端口 (固定单播) */

    const esp_partition_t *run_part = esp_ota_get_running_partition(); /* 查询当前运行的 app 分区 */
    if (run_part) { /* 查询成功 */
        ESP_LOGI(TAG, "running from partition '%s' @0x%08lx (OTA: switch between ota_0/ota_1)", /* 日志: 便于确认 OTA 是否切换成功 */
                 run_part->label, (unsigned long)run_part->address); /* 分区名与地址 */
    } /* 查询分支结束 */

    /* I2S + ACM8815 输出链路初始化 (按配置选择 full/single 与声道) */
    audio_out_init(strcmp(s_cfg.output_mode, "full") == 0, s_cfg.slot_index); /* 初始化输出链路 */

    /* 状态灯: 工作模式常绿 (基准状态); BLE 被手机连接时切蓝色 1Hz 闪烁, 断开回常绿 */
    ws2812_init(LED_GPIO_WS2812); /* 初始化状态灯 (WS2812B) */
    ble_provision_set_conn_cb(on_ble_conn); /* 注册 BLE 连接/断开回调 (切状态灯) */
    xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 1, NULL, 0); /* 状态灯任务 (核0, 优先级1) */

    /* FEC 包缓存: 64 槽 × ~2.3KB ≈ 148KB, 放 PSRAM (calloc 保证 valid=false);
     * 必须在 net/i2s 任务创建前完成 */
    s_cache = heap_caps_calloc(FEC_CACHE_SZ, sizeof(pkt_cache_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_cache);

    /* 环形缓冲: RING_PKTS 包容量, 目标水位 TARGET_WMARK 包 */
    ring_init(RING_PKTS * s_pkt_bytes, s_pkt_bytes); /* 初始化环形缓冲 (容量 RING_PKTS 包) */

    /* ESP32-C5 单核: 所有任务固定 Core 0; 优先级分配 i2s(5) > net(4) > stat(1) */
    xTaskCreatePinnedToCore(network_task, "net", 8192, NULL, 4, &s_h_net, 0); /* 网络接收任务 (核0, 优先级4) */
    xTaskCreatePinnedToCore(i2s_task,     "i2s", 4096, NULL, 5, &s_h_i2s, 0); /* I2S 播放任务 (核0, 优先级5) */
    xTaskCreatePinnedToCore(stat_task,    "stat", 2048, NULL, 1, &s_h_stat, 0); /* 统计打印任务 (核0, 优先级1) */
    xTaskCreatePinnedToCore(discovery_task,"disc", 3072, NULL, 1, &s_h_disc, 0); /* 发现广播任务 (核0, 优先级1) */

    /* BLE 常驻: 工作模式下同样开启配网服务, 手机可随时连接修改配置;
     * 写入新配置后保存 NVS 并自动重启生效 (与配网模式同一流程);
     * BLE 为 2.4GHz, 音频走 5GHz WiFi, 射频无冲突 */
    /* 广播名按 ID+声道拼后缀: "7CH-RECEIVER-<id>-L"; id=0 也显示 (便于确认配网下发的 ID);
     * full(TDM 8ch) 模式无单声道概念, 广播名为 "7CH-RECEIVER-<id>" */
    ble_provision_set_device_id(s_cfg.device_id); /* 先设 ID (决定广播名是否带编号) */
    ble_provision_set_slot(s_full ? -1 : (int)s_slot_index); /* 再设声道后缀 (决定广播名声道部分) */
    ble_provision_start(); /* 启动 BLE 配网服务 (与 WiFi 共存, 非阻塞) */

    /* WiFi OTA: 启动 HTTP 升级服务 (端口 8080), 浏览器上传 build/RECEIVER_HIFI.bin 即可升级;
     * 需要双 app 分区表 (partitions.csv, 8MB flash), 无 OTA 分区时服务仍启动但上传会报错 */
    if (ota_update_start() != ESP_OK) { /* 启动失败不影响音频主链路 */
        ESP_LOGE(TAG, "OTA server start failed (audio keeps running)"); /* 日志 */
    } /* OTA 启动结束 */
} /* 函数结束 */

/* OTA flash 写入期间临时降低 receiver 各业务任务优先级, 给 HTTP flash 写让路
 * pause=true:  i2s(5)→1, net(4)→1, disc(1)→1, stat(1)→1
 * pause=false: 恢复各任务原始优先级
 * (httpd 任务由 ota_update.c 在 handler 内自行提升到 configMAX_PRIORITIES-1, 不需要这里管) */
void receiver_ota_priority(bool pause)
{
    static bool s_last = false; /* 防重复设置: OTA 不会同时写多块 */
    if (pause == s_last) return;
    s_last = pause;
    if (pause) {
        if (s_h_i2s)  vTaskPrioritySet(s_h_i2s,  1);
        if (s_h_net)  vTaskPrioritySet(s_h_net,  1);
        if (s_h_disc) vTaskPrioritySet(s_h_disc, 1);
        if (s_h_stat) vTaskPrioritySet(s_h_stat, 1);
        ESP_LOGW(TAG, "OTA pause: i2s/net/disc/stat → prio 1 (flash write)");
    } else {
        if (s_h_i2s)  vTaskPrioritySet(s_h_i2s,  5);
        if (s_h_net)  vTaskPrioritySet(s_h_net,  4);
        if (s_h_disc) vTaskPrioritySet(s_h_disc, 1);
        if (s_h_stat) vTaskPrioritySet(s_h_stat, 1);
        ESP_LOGI(TAG, "OTA resume: i2s→5, net→4, disc/stat→1");
    }
}
