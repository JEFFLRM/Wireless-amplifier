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

#ifdef __cplusplus /* C++ 兼容: 外部链接 "C" 结束 */
} /* C++ 兼容: 声明域结束 */
#endif /* C++ 兼容: 条件结束 */

#endif /* OTA_UPDATE_H */
