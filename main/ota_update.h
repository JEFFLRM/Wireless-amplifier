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

#ifdef __cplusplus /* C++ 兼容: 外部链接 "C" 开始 */
extern "C" { /* C++ 兼容: 声明进入 C 链接域 */
#endif /* C++ 兼容: 条件结束 */

/** 启动 HTTP OTA 服务 (端口 8080); 重复调用只启动一次
 *  @return ESP_OK 启动成功 / ESP_ERR_INVALID_STATE 已启动 / 其他 httpd 错误码 */
esp_err_t ota_update_start(void); /* 启动 OTA 服务 */

/** 升级页面 (/cfg) 可调参数: FEC 缓存 / 环形缓冲 / 定时播出
 *  标"重启生效"的字段只写 NVS, 重启后由 receiver.c 的 tuning_load 加载 */
typedef struct {
    int cache_sz;       /* FEC 包缓存槽数 (重启生效) */
    int group_rec;      /* FEC 组状态记录数 (重启生效) */
    int wmark;          /* 起播水位 (帧) */
    int high;           /* 积压上限 (帧) */
    int trim;           /* 裁剪目标 (帧) */
    int ring;           /* 环形缓冲容量 (帧, 重启生效) */
    int lead_ms;        /* 播放前瞻 (ms) */
    int dma_ms;         /* I2S DMA 深度 (ms, 重启生效) */
    int late_drop_ms;   /* 迟到丢弃阈值 (ms) */
    int drain_fill;     /* 单次排水静音上限 (帧) */
    int gap_jump;       /* 缺口跳转阈值 (包) */
    int ts_sane_ms;     /* 墙钟判定容差 (ms) */
    int sync_wait_ms;   /* 起播等对时上限 (ms) */
    int ntp_interval_ms;/* NTP 刷新周期 (ms, 重启生效) */
} receiver_tuning_t;

/** 读取当前生效参数 */
void receiver_tuning_get(receiver_tuning_t *t); /* receiver.c 实现 */

/** 校验并保存参数; 可热改项立即生效, 内存/硬件相关项重启生效
 *  @return ESP_OK 成功 / ESP_ERR_INVALID_ARG 参数非法 / NVS 错误码 */
esp_err_t receiver_tuning_set(const receiver_tuning_t *t); /* receiver.c 实现 */

#ifdef __cplusplus /* C++ 兼容: 外部链接 "C" 结束 */
} /* C++ 兼容: 声明域结束 */
#endif /* C++ 兼容: 条件结束 */

#endif /* OTA_UPDATE_H */
