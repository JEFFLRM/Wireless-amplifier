/**
 * ble_provision.h - BLE 一键配网模块 (ESP32-C5 接收端)
 *
 * 基于 NimBLE 实现 GATT 配网服务 + NVS 配置存储。
 * 复用发送端 HIFI/main/ble_provision 的配网框架, 扩展两个配置项:
 *   - slot_index  (uint8, 0~7): 本机负责的声道序号, 默认 0
 *   - output_mode (string): "single"=只输出配置的声道 / "full"=输出全部 8 声道
 * 配网流程: 上电无有效配置 -> 进入 BLE 配网模式 -> 手机写入 JSON 配置
 *           -> 校验 -> 存 NVS -> 通知 ok -> 1s 后重启进入工作模式。
 *
 * 配网 JSON 字段 (Char2 可写):
 *   {"mode":"sta","ssid":"MyRouter5G","password":"12345678",
 *    "slot_index":0,"output_mode":"single","device_id":1}
 *   (音频端口固定 UDP 5004 单播, 音频目的地址由接收端发现广播包通告, 无需配网下发)
 *
 * 依赖: ESP-IDF v5.4+ (CONFIG_BT_ENABLED / CONFIG_BT_NIMBLE_ENABLED)
 */
#ifndef BLE_PROVISION_H /* 头文件防重复包含: 首次定义 */
#define BLE_PROVISION_H /* 头文件防重复包含: 定义完成 */

#include <stdbool.h> /* bool 类型 */
#include <stdint.h> /* uint8_t/uint16_t 等定长类型 */
#include "esp_err.h" /* esp_err_t 错误码类型 */

#ifdef __cplusplus /* C++ 兼容: 外部链接 "C" 开始 */
extern "C" { /* C++ 兼容: 声明进入 C 链接域 */
#endif /* C++ 兼容: 条件结束 */

#define PROV_MODE_LEN   8       /* "ap" / "sta" + NUL */
#define PROV_SSID_LEN   33      /* WiFi SSID 最大 32 字符 + NUL */
#define PROV_PASS_LEN   65      /* WiFi 密码最大 64 字符 + NUL */
#define PROV_OUT_LEN    8       /* "single" / "full" + NUL */

#define PROV_VERSION    0x04    /* 结构版本 (v4: 取消 mcast_ip/mcast_port, 仅单播 UDP 5004) */

/** 配网配置结构 (NVS blob 持久化) */
typedef struct { /* 配网配置结构体定义开始 */
    uint8_t  version;           /* 结构版本, 当前 PROV_VERSION */
    char     mode[PROV_MODE_LEN];   /* "ap"=连发送端 SoftAP / "sta"=连路由器 */
    char     ssid[PROV_SSID_LEN]; /* WiFi 密码 (字符串) */
    char     password[PROV_PASS_LEN]; /* WiFi 密码 (字符串) */
    uint8_t  channel;           /* 仅发送端 SoftAP 需要, 接收端忽略; 保留兼容 */
    uint8_t  slot_index;        /* 本机声道序号 0~7 (L,R,C,LFE,SL,SR,SBL,SBR), 默认 0 */
    char     output_mode[PROV_OUT_LEN]; /* "single"=只输出 slot_index 声道 / "full"=输出 8 声道 */
    uint8_t  device_id;         /* 设备编号 0~255, 由 PC/手机配网 JSON "device_id" 下发;
                                   发现广播包按此值标识本机, 同网段多台接收端必须各不相同 */
} prov_config_t; /* 配网配置结构体定义结束 */

/** 保存配置到 NVS (调用前先校验) */
esp_err_t prov_config_save(const prov_config_t *cfg); /* 保存配网配置到 NVS (校验通过才保存) */

/** 从 NVS 加载配置; 无配置返回 ESP_ERR_NVS_NOT_FOUND */
esp_err_t prov_config_load(prov_config_t *cfg); /* 从 NVS 加载配网配置 */

/** 清空 NVS 配置 */
esp_err_t prov_config_clear(void); /* 清除 NVS 中的配网配置 (长按 BOOT 触发) */

/** 校验配置: version 正确且 mode 为 ap/sta 且 ssid 非空且 slot_index<=7 且 output_mode 合法 */
bool prov_config_valid(const prov_config_t *cfg); /* 校验配网配置合法性 */

/** 启动 BLE 配网模式 (阻塞直至配网完成后重启, 不返回) */
void ble_provision_start(void); /* 启动 BLE 配网流程 (阻塞直到配置完成) */

/** BLE 连接状态回调: connected=true 连接建立, false 断开 */
typedef void (*prov_conn_cb_t)(bool connected); /* 连接状态回调类型定义 */

/** 注册 BLE 连接/断开回调 (可选, 供连接提示音等使用); 传 NULL 取消注册 */
void ble_provision_set_conn_cb(prov_conn_cb_t cb); /* 注册连接/断开回调 */

/** 查询当前 BLE 是否已连接 (用于持续提示音: 连接时一直响, 断开即停) */
bool ble_provision_is_connected(void); /* 返回当前 BLE 连接状态 */

/** 配网成功回调: 配置校验通过并写入 NVS 成功后调用 (重启前) */
typedef void (*prov_done_cb_t)(void); /* 配网成功回调类型定义 */

/** 注册配网成功回调 (可选, 供状态灯转常绿等使用); 传 NULL 取消注册 */
void ble_provision_set_done_cb(prov_done_cb_t cb); /* 注册配网成功回调 */

/** 取新配置覆盖保存前的旧配置快照 (供在旧 WiFi 上先发"停止发送"通告, 再重启连新 WiFi);
 *  首次配网无旧配置返回 false */
bool ble_provision_get_prev_config(prov_config_t *out); /* 取旧配置快照 */

/** 更新 BLE 广播厂商数据中携带的本机 IPv4 (网络序, 0=未联网);
 *  手机扫描广播包即可读到本机 IP; WiFi 拿到/丢失 IP 时调用 */
void ble_provision_set_ip(uint32_t ip_be); /* 更新广播携带的 IP */

/** 设置声道序号用于广播名后缀: 0~7 -> 广播名 "7CH-RECEIVER-ID-L/R/C/LFE/SL/SR/SBL/SBR";
 *  -1 (默认) 不加声道后缀, 广播名保持 "7CH-RECEIVER-ID"。需在 ble_provision_start 之前调用 */
void ble_provision_set_slot(int slot_index); /* 声道后缀: L/R/C/LFE/SL/SR/SBL/SBR */

/** 设置设备编号用于广播名前缀 (在声道号之前):
 *  广播名始终带 ID, 例如 "7CH-RECEIVER-1-L"; id = 0 时同样显示 "7CH-RECEIVER-0-L",
 *  与 WiFi hostname 命名规则完全一致 (0 也显示, 便于确认 ID 已生效)
 *  需在 ble_provision_set_slot 之前或之后调用均可 */
void ble_provision_set_device_id(uint8_t device_id); /* 设备编号 0~255 */

#ifdef __cplusplus /* C++ 兼容: 关闭外部链接域 */
} /* C++ 兼容: 条件结束 */
#endif /* 头文件防重复包含: 结束 */

#endif /* BLE_PROVISION_H */
