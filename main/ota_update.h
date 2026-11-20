/**
 * ota_update.h - WiFi OTA 固件升级接口 (HTTP 上传)
 *
 * 用法: WiFi 已连接并拿到 IP 后调用 ota_update_start();
 *       浏览器打开 http://<设备IP>:8080/ 选择 RECEIVER_HIFI.bin 上传即可升级。
 *
 * 前提: 分区表必须包含两个 app 分区 (ota_0/ota_1) + otadata,
 *       因此需要 4MB 以上 flash (本工程按 8MB 配置, 见 partitions.csv)。
 */
#ifndef OTA_UPDATE_H /* 头文件防重复包含: 首次定义 */
#define OTA_UPDATE_H /* 头文件防重复包含: 定义完成 */

#include "esp_err.h" /* esp_err_t */
#include <stdbool.h> /* bool */
#include <stdint.h>  /* int32_t / uint32_t */

#ifdef __cplusplus /* C++ 兼容: 外部链接 "C" 开始 */
extern "C" { /* C++ 兼容: 声明进入 C 链接域 */
#endif /* C++ 兼容: 条件结束 */

/** 启动 HTTP OTA 服务 (端口 8080); 重复调用只启动一次
 *  @return ESP_OK 启动成功 / ESP_ERR_INVALID_STATE 已启动 / 其他 httpd 错误码 */
esp_err_t ota_update_start(void); /* 启动 OTA 服务 */

/** 运行状态快照 (升级网页 /stat 显示用, receiver.c 实现) */
typedef struct {
    bool     synced;          /* 时钟是否已同步 (SNTP) */
    char     sntp[32];        /* 当前 NTP server */
    int32_t  ntp_offset_ms;   /* 墙钟 - 发端 RTP ts (ms) */
    uint32_t ntp_sync_cnt;    /* 累计对时成功次数 */
    uint32_t rx_pps;          /* 收包速率 (数据包/s) */
    uint32_t rx_total;        /* 累计数据包 (1ms + 批量) */
    uint32_t rx_fec;          /* 累计 FEC 包 */
    uint32_t recovered;       /* FEC 恢复成功包数 */
    int32_t  late_ms;         /* 当前播放延迟 late (ms, 负=超前) */
    int32_t  late_min;        /* 统计窗内 late 最小值 */
    int32_t  late_max;        /* 统计窗内 late 最大值 */
    int32_t  jitter_ms;       /* 网络抖动 (ms) */
    uint32_t buffer_ms;       /* ring 缓冲深度 (ms) */
    uint32_t silence_ms;      /* 累计补静音 (ms) */
    uint32_t underrun;        /* 欠载次数 */
    uint32_t late_drop;       /* 因迟到丢弃帧数 */
    bool     use_wall;        /* true=墙钟模式 / false=本地锚定 */
} receiver_stats_t;

/** 读取运行状态快照 */
void receiver_stats_get(receiver_stats_t *st); /* receiver.c 实现 */

#ifdef __cplusplus /* C++ 兼容: 外部链接 "C" 结束 */
} /* C++ 兼容: 声明域结束 */
#endif /* C++ 兼容: 条件结束 */

#endif /* OTA_UPDATE_H */
