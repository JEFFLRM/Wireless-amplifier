/**
 * ble_provision.c - BLE 一键配网模块 (ESP32-C5 接收端)
 *
 * 基于 NimBLE 的 GATT 配网服务 (复用发送端框架):
 *   Service  0000ff00-0000-1000-8000-00805f9b34fb
 *     Char1 设备信息 0000ff01-...  (只读, JSON: device_role/fw_version/wifi_status)
 *     Char2 配网配置 0000ff02-...  (可写, JSON: mode/ssid/password/channel/slot_index/output_mode/device_id)
 *     Char3 配网状态 0000ff03-...  (通知, receiving / ok / error:<原因>)
 *
 * 配网流程:
 *   上电无有效配置 -> 关 WiFi 开 BLE 广播 -> 手机连接并写入 JSON
 *   -> 解析校验 -> 存 NVS -> 通知 ok -> 延时 3s -> esp_restart()
 *     (延时窗口内接收端在当前 WiFi 上直接广播 tx_enable=0 的"停止发送"通告, 再重启用新配置连 WiFi)
 *   -> 重启后按 NVS 配置进入工作模式 (工作模式 BLE 常驻, 与 5GHz WiFi 共存, 可随时改配置)
 *
 * 扩展项 (相对发送端):
 *   - slot_index  (uint8 0~7) : 本机声道序号, JSON 数字字段 "slot_index"
 *   - output_mode (string)    : "single" / "full", JSON 字符串字段 "output_mode"
 *   - device_id   (uint8 0~255): 设备编号, JSON 数字字段 "device_id" (发现广播包按此标识本机)
 *
 * 注意: ESP32-C5 的 BLE 在 2.4GHz、WiFi 5GHz 为不同频段;
 *       配网模式仅开 BLE, 工作模式仅开 WiFi, 避免射频资源冲突。
 *
 * 依赖: ESP-IDF v5.4+ NimBLE (CONFIG_BT_ENABLED / CONFIG_BT_NIMBLE_ENABLED)
 */
#include <string.h> /* 字符串操作库 (strlen/strncpy) */
#include <stdlib.h> /* 标准库 (atoi 解析字符串形式的设备编号) */
#include <stdio.h> /* 标准 IO (snprintf) */
#include "freertos/FreeRTOS.h" /* FreeRTOS 内核头文件 */
#include "freertos/task.h" /* 任务 API (xTaskCreate) */
#include "esp_log.h" /* ESP 日志 */
#include "esp_system.h" /* ESP 系统 API (esp_restart/abort) */
#include "esp_timer.h" /* 定时器: 广播失败带退避重试 */
#include "nvs_flash.h" /* NVS flash 存储 */
#include "nvs.h" /* NVS 键值操作 (nvs_get_blob 等) */
#include "cJSON.h" /* cJSON: 解析手机下发的配网 JSON */

#include "nimble/nimble_port.h" /* NimBLE 移植层 */
#include "nimble/nimble_port_freertos.h" /* NimBLE FreeRTOS 移植层 */
#include "host/ble_hs.h" /* NimBLE 主机栈 (GATT/GAP) */
#include "host/util/util.h" /* NimBLE 工具函数 */
#include "services/gap/ble_svc_gap.h" /* GAP 服务 (广播/设备名) */
#include "services/gatt/ble_svc_gatt.h" /* GATT 服务 (特征值表) */

#include "ble_provision.h" /* 本模块头文件 (prov_config_t 及接口) */

#define TAG "ble_prov" /* 日志标签 */

/* ============ BLE 服务/特征 UUID ============ */
#define PROV_BASE_UUID_128(lo)                                   \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,              \
    0x00, 0x10, 0x00, 0x00, lo, 0xff, 0x00, 0x00 /* UUID 后 8 字节 (含自定义 lo 字节) */

static const ble_uuid128_t gatt_svc_uuid = /* GATT 服务 UUID (128 位) */
    BLE_UUID128_INIT(PROV_BASE_UUID_128(0x00));   /* 0000ff00-... */
static const ble_uuid128_t chr_dev_info_uuid = /* 设备信息特征 UUID */
    BLE_UUID128_INIT(PROV_BASE_UUID_128(0x01));   /* 0000ff01-... 设备信息(只读) */
static const ble_uuid128_t chr_prov_cfg_uuid = /* 配网配置特征 UUID */
    BLE_UUID128_INIT(PROV_BASE_UUID_128(0x02));   /* 0000ff02-... 配网配置(可写) */
static const ble_uuid128_t chr_prov_status_uuid = /* 配网状态特征 UUID */
    BLE_UUID128_INIT(PROV_BASE_UUID_128(0x03));   /* 0000ff03-... 配网状态(通知) */

/* ============ 常量 ============ */
#define DEVICE_NAME         "7CH-RECEIVER"        /* 广播名基础部分 (不可变), 手机可据此识别 */
/* 广播名后缀: 按声道序号拼接, slot0..7 = L/R/C/LFE/SL/SR/SBL/SBR (与发送端 CHANNEL_NAMES 一致) */
static const char *const SLOT_SUFFIX[8] = { "FL", "FR", "C", "SW", "SL", "SR", "BL", "BR" }; /* 声道名表 */
static char s_dev_name[32] = DEVICE_NAME; /* 实际广播名 (默认无后缀, ble_provision_set_slot 可拼接) */
static uint8_t s_device_id = 0; /* 设备编号 (配网 JSON "device_id" 下发, 0~255) */
static uint32_t s_adv_ip = 0;   /* 广播厂商数据里携带的本机 IPv4 (网络序; 0=未联网) */

static void start_adv(void); /* 前向声明: IP 变化时需停播后重新组包启动 */

void ble_provision_set_ip(uint32_t ip_be) /* 更新广播里的本机 IP (WiFi 拿到/丢失 IP 时调用) */
{ /* 函数体开始 */
    if (ip_be == s_adv_ip) return; /* IP 未变化: 无需重设广播 */
    s_adv_ip = ip_be; /* 记录新 IP (网络序) */
    if (ble_hs_synced() && ble_gap_adv_active()) { /* 广播进行中: 停播后按新数据重启 (host 未同步时跳过) */
        ble_gap_adv_stop(); /* 停止广播 */
        start_adv(); /* 用新 IP 重新组包启动 */
    } /* 广播中分支结束 */
    ESP_LOGI(TAG, "BLE adv IP set to %u.%u.%u.%u", /* 日志: 广播携带的 IP */
             (unsigned)ip_be & 0xFF, (unsigned)(ip_be >> 8) & 0xFF, /* 前两字节 */
             (unsigned)(ip_be >> 16) & 0xFF, (unsigned)(ip_be >> 24) & 0xFF); /* 后两字节 */
} /* 函数结束 */

/* 拼广播名: 固定 "基础名-ID-声道"; 无声道时 "基础名-ID" (ID=0 也显示, 便于确认 ID 已生效) */
static void rebuild_dev_name(int slot_index)
{ /* 函数体开始 */
    if (slot_index >= 0 && slot_index <= 7) { /* 合法声道序号 */
        snprintf(s_dev_name, sizeof(s_dev_name), "%s-%u-%s", DEVICE_NAME, s_device_id, SLOT_SUFFIX[slot_index]); /* "7CH-RECEIVER-1-L" */
    } else { /* -1 等: 没有声道信息, 只拼 ID */
        snprintf(s_dev_name, sizeof(s_dev_name), "%s-%u", DEVICE_NAME, s_device_id); /* "7CH-RECEIVER-1" */
    } /* 分支结束 */
} /* 函数结束 */

void ble_provision_set_device_id(uint8_t device_id) /* 设置设备编号 -> 广播名拼 ID (start 前调用) */
{ /* 函数体开始 */
    s_device_id = device_id; /* 保存编号 */
    rebuild_dev_name(-1); /* 先按 slot=-1 重建一次 (还没 set_slot 时只拼 ID) */
    ESP_LOGI(TAG, "BLE device id set to %u", s_device_id); /* 日志 */
} /* 函数结束 */

void ble_provision_set_slot(int slot_index) /* 设置声道序号 -> 广播名拼 ID+声道 (start 前调用, 在 set_device_id 之后) */
{ /* 函数体开始 */
    rebuild_dev_name(slot_index); /* 统一重建: ID+声道 */
    ESP_LOGI(TAG, "BLE device name set to '%s' (id=%u slot=%d)", s_dev_name, s_device_id, slot_index); /* 日志: 确认广播名 */
} /* 函数结束 */
#define FW_VERSION          "1.0.0" /* 固件版本号 */
#define DEVICE_ROLE         "receiver" /* 设备角色 (receiver 接收端) */
#define DEFAULT_AP_CHANNEL  36 /* 默认 AP 信道 (36 = 5GHz 信道) */
#define RESTART_DELAY_MS    3000 /* 配网成功后重启延时 (毫秒): 需大于"停止发送"通告耗时
                                    (当前 WiFi 上连发 3 包 ~0.5s) + GATT ok 通知发出时间 */

#define NVS_NAMESPACE       "prov" /* NVS 命名空间: 配网数据 */
#define NVS_KEY_CFG         "cfg" /* NVS 键名: 配置 blob */

/* ============ 状态/句柄 ============ */
static uint16_t g_conn_handle      = BLE_HS_CONN_HANDLE_NONE; /* 当前连接句柄 (无连接时为 NONE) */
static uint16_t g_status_val_handle = 0;          /* Char3 值句柄, 用于通知 */
static prov_conn_cb_t g_conn_cb    = NULL;        /* 连接/断开回调 (由调用方注册, 用于连接提示音等) */
static prov_done_cb_t g_done_cb    = NULL;        /* 配网成功回调 (由调用方注册, 用于状态灯转常绿等) */

/* 旧配置快照: 新配置写入 NVS 前捕获, 供"先在旧 WiFi 发停止通告, 再重启连新 WiFi"流程使用 */
static prov_config_t s_prev_cfg;        /* 旧配置内容 */
static bool          s_has_prev_cfg = false; /* 旧配置是否有效 (首次配网为 false) */

bool ble_provision_get_prev_config(prov_config_t *out) /* 取旧配置快照: 有旧配置返回 true */
{ /* 函数体开始 */
    if (!s_has_prev_cfg || out == NULL) return false; /* 无快照/参数非法 */
    *out = s_prev_cfg; /* 拷出快照 */
    return true; /* 返回成功 */
} /* 函数结束 */

void ble_provision_set_conn_cb(prov_conn_cb_t cb) /* 注册 BLE 连接/断开回调 */
{ /* 函数体开始 */
    g_conn_cb = cb; /* 保存回调指针 (可为 NULL 取消) */
} /* 函数结束 */

void ble_provision_set_done_cb(prov_done_cb_t cb) /* 注册配网成功回调 */
{ /* 函数体开始 */
    g_done_cb = cb; /* 保存回调指针 (可为 NULL 取消) */
} /* 函数结束 */

bool ble_provision_is_connected(void) /* 查询当前 BLE 是否已连接 */
{ /* 函数体开始 */
    return g_conn_handle != BLE_HS_CONN_HANDLE_NONE; /* 句柄有效即已连接 */
} /* 函数结束 */


/* ============ 通知 ============ */
static void notify_status(const char *status) /* 通过状态特征通知手机当前配网状态 */
{ /* 函数体开始 */
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || g_status_val_handle == 0) { /* 无连接或无状态特征句柄: 无法通知 */
        return;                                   /* 未连接或未订阅, 忽略 */
    } /* 空分支结束 */
    struct os_mbuf *om = ble_hs_mbuf_from_flat(status, (uint16_t)strlen(status)); /* 将状态字符串封装为 NimBLE 缓冲区 */
    if (om) { /* 封装成功 */
        ble_gatts_notify_custom(g_conn_handle, g_status_val_handle, om); /* 发送自定义 GATT 通知 */
    } /* 通知分支结束 */
} /* 函数结束 */

/* 配网成功后延时重启 (不在 BLE host 回调上下文中阻塞) */
static void restart_task(void *arg) /* 重启延时任务: 等待后重启系统 */
{ /* 任务入口 */
    vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS)); /* 等待 RESTART_DELAY_MS (让通知有时间发出) */
    esp_restart(); /* 重启系统 */
    (void)arg; /* 忽略入参 (不可达) */
} /* 函数结束 */

static void schedule_restart(void) /* 创建重启延时任务 */
{ /* 函数体开始 */
    xTaskCreate(restart_task, "prov_restart", 2048, NULL, 1, NULL); /* 创建独立任务执行重启 (避免阻塞 GATT 回调) */
} /* 函数结束 */

/* ============ NVS 配置接口 ============ */
static esp_err_t prov_nvs_set(const prov_config_t *cfg) /* 将配网配置写入 NVS (blob 形式) */
{ /* 函数体开始 */
    nvs_handle_t h; /* NVS 句柄 */
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h); /* 打开命名空间 (读写模式) */
    if (err != ESP_OK) return err; /* 打开失败直接返回错误 */
    err = nvs_set_blob(h, NVS_KEY_CFG, cfg, sizeof(*cfg)); /* 以 blob 方式写入整个配置结构 */
    if (err == ESP_OK) err = nvs_commit(h); /* 写入成功则提交到 flash */
    nvs_close(h); /* 关闭句柄 */
    return err; /* 返回结果 */
} /* 函数结束 */

esp_err_t prov_config_save(const prov_config_t *cfg) /* 对外接口: 保存配网配置 (先校验) */
{ /* 函数体开始 */
    if (!cfg || !prov_config_valid(cfg)) { /* 指针为空或配置不合法 */
        return ESP_ERR_INVALID_ARG; /* 返回参数错误 */
    } /* 非法分支结束 */
    return prov_nvs_set(cfg); /* 调用内部 NVS 写入 */
} /* 函数结束 */

esp_err_t prov_config_load(prov_config_t *cfg) /* 对外接口: 从 NVS 加载配网配置 */
{ /* 函数体开始 */
    nvs_handle_t h; /* NVS 句柄 */
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h); /* 打开命名空间 (只读模式) */
    if (err != ESP_OK) return err; /* 打开失败直接返回错误 */
    size_t len = sizeof(*cfg); /* blob 期望长度 = 配置结构大小 */
    err = nvs_get_blob(h, NVS_KEY_CFG, cfg, &len); /* 读取 blob 到配置结构 */
    nvs_close(h); /* 关闭句柄 */
    return err; /* 返回读取结果 */
} /* 函数结束 */

esp_err_t prov_config_clear(void) /* 对外接口: 清除 NVS 中的配网配置 */
{ /* 函数体开始 */
    nvs_handle_t h; /* NVS 句柄 */
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h); /* 打开命名空间 (读写模式) */
    if (err != ESP_OK) return err; /* 打开失败直接返回错误 */
    err = nvs_erase_key(h, NVS_KEY_CFG); /* 删除配置键 */
    if (err == ESP_OK) err = nvs_commit(h); /* 删除成功则提交 */
    nvs_close(h); /* 关闭句柄 */
    return err; /* 返回结果 */
} /* 函数结束 */

bool prov_config_valid(const prov_config_t *cfg) /* 校验配置合法性 (供保存/加载后检查) */
{ /* 函数体开始 */
    if (!cfg || cfg->version != PROV_VERSION) return false; /* 空指针或版本不匹配: 不合法 */
    if (strcmp(cfg->mode, "ap") != 0 && strcmp(cfg->mode, "sta") != 0) return false; /* 模式必须是 ap 或 sta */
    if (cfg->ssid[0] == '\0') return false; /* SSID 不能为空 */
    if (cfg->slot_index > 7) return false; /* 声道索引范围 0-7 */
    if (strcmp(cfg->output_mode, "single") != 0 && strcmp(cfg->output_mode, "full") != 0) { /* 输出模式必须是 single 或 full */
        return false; /* 输出模式非法返回 false */
    } /* 非法分支结束 */
    return true; /* 全部通过返回 true */
} /* 函数结束 */

/* ============ Char1 设备信息 (只读) ============ */
static int chr_dev_info_access(uint16_t conn_handle, uint16_t attr_handle, /* 设备信息特征回调: 手机读取设备角色/固件/配网状态 */
                               struct ble_gatt_access_ctxt *ctxt, void *arg) /* 函数体开始 */
{ /* 函数体开始 */
    (void)conn_handle; (void)attr_handle; (void)arg; /* 忽略未使用参数 */
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) { /* 只响应读操作 */
        char json[160]; /* 设备信息 JSON 缓冲区 */
        int n = snprintf(json, sizeof(json), /* 格式化 JSON 字符串 */
                         "{\"device_role\":\"%s\",\"fw_version\":\"%s\",\"wifi_status\":\"provisioning\"}", /* JSON 内容: 设备角色/固件版本/配网状态 */
                         DEVICE_ROLE, FW_VERSION); /* 填充参数: 角色与版本 */
        if (n < 0) n = 0; /* snprintf 出错归零 */
        if (n >= (int)sizeof(json)) n = (int)sizeof(json) - 1; /* 防止截断 (保留结尾 0 位置) */
        return os_mbuf_append(ctxt->om, json, (uint16_t)n); /* 将 JSON 字符串附加到 GATT 响应缓冲区 */
    } /* 读操作分支结束 */
    return BLE_ATT_ERR_UNLIKELY; /* 非读操作返回错误 */
} /* 函数结束 */

/* ============ Char2 配网配置 (可写 JSON) ============ */
static void json_str_to_fixed(char *dst, size_t dst_len, cJSON *item) /* 工具函数: 将 cJSON 字符串安全拷贝到定长数组 */
{ /* 函数体开始 */
    if (item && cJSON_IsString(item)) { /* 节点存在且为字符串 */
        strncpy(dst, item->valuestring, dst_len - 1); /* 拷贝字符串 (留 1 字节给结尾 0) */
        dst[dst_len - 1] = '\0'; /* 强制结尾 0 */
    } else { /* 合法分支结束 */
        dst[0] = '\0'; /* 非法则置空串 */
    } /* 非法分支结束 */
} /* 函数结束 */

static int chr_prov_cfg_access(uint16_t conn_handle, uint16_t attr_handle, /* 配网配置特征回调: 手机写入 JSON 完成配网 */
                               struct ble_gatt_access_ctxt *ctxt, void *arg) /* 函数体开始 */
{ /* 函数体开始 */
    (void)attr_handle; (void)arg; /* 忽略未使用参数 */
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) { /* 只接受写操作 */
        return BLE_ATT_ERR_UNLIKELY; /* 非写操作返回错误 */
    } /* 非法操作分支结束 */

    g_conn_handle = conn_handle; /* 记录连接句柄 (供状态通知用) */
    notify_status("receiving"); /* 通知手机: 正在接收配置 */

    /* 读取写入的 JSON */
    char buf[512]; /* 接收缓冲区 (最大 512 字节) */
    uint16_t len = (uint16_t)OS_MBUF_PKTLEN(ctxt->om); /* 取手机写入的数据长度 */
    if (len == 0 || len >= sizeof(buf)) { /* 空数据或超长 */
        notify_status("error:payload too long"); /* 通知手机错误 */
        return BLE_ATT_ERR_INSUFFICIENT_RES; /* 返回资源不足错误 */
    } /* 非法长度分支结束 */
    if (os_mbuf_copydata(ctxt->om, 0, len, buf) != 0) { /* 从 mbuf 拷贝写入数据到缓冲区 */
        notify_status("error:read fail"); /* 拷贝失败通知手机 */
        return BLE_ATT_ERR_UNLIKELY; /* 返回错误 */
    } /* 拷贝失败分支结束 */
    buf[len] = '\0'; /* 添加字符串结尾 0 */

    /* 解析 JSON */
    cJSON *root = cJSON_Parse(buf); /* 解析配网 JSON */
    if (!root) { /* 解析失败 */
        notify_status("error:invalid json"); /* 通知手机 JSON 非法 */
        return 0;                                 /* 已通知错误, 允许重写 */
    } /* 空分支结束 (释放由 cJSON_Delete 处理) */

    prov_config_t cfg = { 0 }; /* 新建配置结构并清零 */
    cfg.version = PROV_VERSION; /* 写入配置版本号 */

    json_str_to_fixed(cfg.mode, sizeof(cfg.mode), cJSON_GetObjectItem(root, "mode")); /* 解析 mode 字段 (ap/sta) */
    json_str_to_fixed(cfg.ssid, sizeof(cfg.ssid), cJSON_GetObjectItem(root, "ssid")); /* 解析 ssid 字段 */
    json_str_to_fixed(cfg.password, sizeof(cfg.password), cJSON_GetObjectItem(root, "password")); /* 解析 password 字段 */
    json_str_to_fixed(cfg.output_mode, sizeof(cfg.output_mode), cJSON_GetObjectItem(root, "output_mode")); /* 解析 output_mode 输出模式 */

    cJSON *ch    = cJSON_GetObjectItem(root, "channel"); /* 解析 channel 字段 (AP 信道) */
    cJSON *slot  = cJSON_GetObjectItem(root, "slot_index"); /* 解析 slot_index 声道索引 */
    /* 设备编号: 兼容多种键名与类型 (配网端可能下发 device_id / id / deviceId / ID,
     * 值可能是数字 1 或字符串 "1"), 避免字段名不一致导致 ID 静默变 0 */
    cJSON *devid = cJSON_GetObjectItem(root, "device_id"); /* 首选键名 device_id */
    if (!devid) devid = cJSON_GetObjectItem(root, "id"); /* 兼容 id */
    if (!devid) devid = cJSON_GetObjectItem(root, "deviceId"); /* 兼容驼峰 deviceId */
    if (!devid) devid = cJSON_GetObjectItem(root, "ID"); /* 兼容大写 ID */
    cfg.channel = (ch && cJSON_IsNumber(ch)) ? (uint8_t)ch->valuedouble : 0; /* 数字节点才取值, 否则置 0 */
    cfg.slot_index = (slot && cJSON_IsNumber(slot)) ? (uint8_t)slot->valuedouble : 0; /* 声道索引取值 (0 表示未配置) */
    if (devid) { /* 下发了设备编号字段 (任意兼容键名) */
        int v = -1; /* 解析结果 */
        if (cJSON_IsNumber(devid)) v = (int)devid->valuedouble; /* 数字: 直接取整 */
        else if (cJSON_IsString(devid) && devid->valuestring) v = atoi(devid->valuestring); /* 字符串: 转整数 (非数字串得 0) */
        if (v < 0 || v > 255) { /* 超出 0~255 范围: 拒绝配网, 防止 ID 冲突 */
            notify_status("error:device_id must be 0-255"); /* 通知手机 ID 非法 */
            cJSON_Delete(root); /* 释放 JSON 树 */
            return 0; /* 直接返回, 不保存不重启 */
        } /* 非法分支结束 */
        cfg.device_id = (uint8_t)v; /* 合法: 保存设备编号 */
    } else { /* 未下发: 缺省 0 */
        cfg.device_id = 0; /* 默认编号 0 */
    } /* 缺省分支结束 */

    /* 缺省值补全 */
    if (cfg.channel == 0)        cfg.channel = DEFAULT_AP_CHANNEL; /* 信道缺省时用默认值 */
    if (cfg.output_mode[0] == '\0') strncpy(cfg.output_mode, "single", sizeof(cfg.output_mode) - 1); /* 输出模式缺省时用 single */

    cJSON_Delete(root); /* 释放 JSON 解析树 */

    if (!prov_config_valid(&cfg)) { /* 校验最终配置 */
        notify_status("error:mode ap/sta + ssid required; slot_index 0-7; output_mode single/full; device_id 0-255"); /* 通知手机校验错误信息 */
        return 0; /* 返回 (写操作成功但未保存) */
    } /* 校验失败分支结束 */

    esp_err_t err; /* 保存结果 */
    /* 覆盖保存前快照旧配置: 供接收端先在旧 WiFi 上广播 tx_enable=0"停止发送"通告,
     * 再重启连新 WiFi (首次配网无旧配置时快照无效, 接收端跳过通告) */
    s_has_prev_cfg = (prov_config_load(&s_prev_cfg) == ESP_OK && prov_config_valid(&s_prev_cfg)); /* 快照旧配置 */
    ESP_LOGI(TAG, "prev config %s (stop announce %s)", /* 日志: 是否有可通告的旧配置 */
             s_has_prev_cfg ? "captured" : "absent", /* 快照状态 */
             s_has_prev_cfg ? "will run on old WiFi" : "skipped"); /* 通告与否 */

    err = prov_config_save(&cfg); /* 保存配置到 NVS */
    if (err != ESP_OK) { /* 保存失败 */
        notify_status("error:nvs save fail"); /* 通知手机存储错误 */
        return 0; /* 返回 */
    } /* 保存失败分支结束 */

    ESP_LOGI(TAG, "provision ok: mode=%s ssid=%s ch=%u slot=%u id=%u out=%s", /* 日志: 输出配网成功摘要 */
             cfg.mode, cfg.ssid, cfg.channel, /* 日志参数: 模式/SSID/信道 */
             cfg.slot_index, cfg.device_id, cfg.output_mode); /* 日志参数: 声道索引/设备编号/输出模式 */
    notify_status("ok"); /* 通知手机配网成功 */
    if (g_done_cb) g_done_cb(); /* 通知调用方: 配网完成 (状态灯转常绿等) */
    schedule_restart();                           /* 1s 后重启进入工作模式 */
    return 0; /* 返回成功 */
} /* 函数结束 */

/* ============ Char3 配网状态 (通知) ============ */
static int chr_prov_status_access(uint16_t conn_handle, uint16_t attr_handle, /* 状态特征回调: 手机读取当前配网状态 */
                                  struct ble_gatt_access_ctxt *ctxt, void *arg) /* 函数体开始 */
{ /* 函数体开始 */
    (void)conn_handle; (void)arg; /* 忽略未使用参数 */
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) { /* 只响应读操作 */
        g_status_val_handle = attr_handle; /* 记录状态特征句柄 (供 notify_status 使用) */
        const char *st = "idle"; /* 初始状态: idle */
        return os_mbuf_append(ctxt->om, st, (uint16_t)strlen(st)); /* 返回状态字符串 */
    } /* 读操作分支结束 */
    return BLE_ATT_ERR_UNLIKELY; /* 非读操作返回错误 */
} /* 函数结束 */

/* ============ GATT 服务定义 ============ */
static const struct ble_gatt_svc_def gatt_svcs[] = { /* GATT 服务定义表: 一个主服务含 3 个特征 */
    { /* 服务定义开始 */
        .type = BLE_GATT_SVC_TYPE_PRIMARY, /* 服务类型: 主服务 */
        .uuid = &gatt_svc_uuid.u, /* 服务 UUID */
        .characteristics = (struct ble_gatt_chr_def[]) { /* 特征数组开始 */
            { /* 特征 1 定义开始 */
                .uuid = &chr_dev_info_uuid.u, /* 特征 1 UUID: 设备信息 */
                .access_cb = chr_dev_info_access, /* 特征 1 回调: 设备信息读取 */
                .flags = BLE_GATT_CHR_F_READ, /* 特征 1 权限: 可读 */
            }, /* 特征 1 定义结束 */
            { /* 特征 2 定义开始 */
                .uuid = &chr_prov_cfg_uuid.u, /* 特征 2 UUID: 配网配置 */
                .access_cb = chr_prov_cfg_access, /* 特征 2 回调: 接收配网 JSON */
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP, /* 特征 2 权限: 可写 (含无响应写) */
            }, /* 特征 2 定义结束 */
            { /* 特征 3 定义开始 */
                .uuid = &chr_prov_status_uuid.u, /* 特征 3 UUID: 配网状态 */
                .access_cb = chr_prov_status_access, /* 特征 3 回调: 状态读取 */
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, /* 特征 3 权限: 可读 + 通知 */
            }, /* 特征 3 定义结束 */
            { 0 }, /* 特征数组结束标记 */
        }, /* 特征数组结束 */
    }, /* 服务定义结束 */
    { 0 }, /* 服务表结束标记 */
}; /* 服务表定义结束 */

/* gap_event_handler 定义于 GAP 事件段 (下方), start_adv 先使用, 需前向声明 */
static int gap_event_handler(struct ble_gap_event *event, void *arg); /* GAP 事件回调前向声明 */

/* ============ 广播 ============ */
#define ADV_RETRY_BASE_MS   1000 /* 广播失败重试基础间隔 (毫秒) */
#define ADV_RETRY_MAX_MS    10000 /* 广播失败重试最大间隔 (毫秒) */
#define ADV_RETRY_MAX       8   /* 广播失败最大重试次数, 超限重启 */

static uint32_t s_adv_retry_cnt = 0; /* 广播连续失败计数 */
static esp_timer_handle_t s_adv_retry_timer = NULL; /* 广播重试定时器 */

static void start_adv(void); /* 前向声明 (重试回调使用) */

static void adv_retry_cb(void *arg) /* 广播重试定时器回调: 到点后再次尝试广播 */
{
    (void)arg; /* 忽略参数 */
    start_adv(); /* 重新启动广播 */
}

static void schedule_adv_retry(int rc) /* 广播失败: 打印错误并安排带退避的重试 */
{
    s_adv_retry_cnt++; /* 失败计数递增 */
    if (s_adv_retry_cnt >= ADV_RETRY_MAX) { /* 超过最大重试次数 */
        ESP_LOGE(TAG, "adv start failed %u times (rc=%d), restarting to retry provisioning", s_adv_retry_cnt, rc); /* 显式报错 */
        esp_restart(); /* 重启重新进入配网 */
        return; /* 不可达 */
    } /* 超限分支结束 */
    uint32_t delay_ms = ADV_RETRY_BASE_MS << (s_adv_retry_cnt - 1); /* 退避: 1s 2s 4s 8s... */
    if (delay_ms > ADV_RETRY_MAX_MS) delay_ms = ADV_RETRY_MAX_MS; /* 封顶 10s */
    if (s_adv_retry_timer == NULL) { /* 定时器未创建则创建 */
        esp_timer_create_args_t args = { /* 定时器创建参数 */
            .callback = adv_retry_cb, /* 回调函数 */
            .arg = NULL, /* 无参数 */
            .name = "adv_retry", /* 定时器名称 */
        }; /* 参数结构结束 */
        if (esp_timer_create(&args, &s_adv_retry_timer) != ESP_OK) { /* 创建失败 */
            ESP_LOGE(TAG, "adv retry timer create failed"); /* 报错 */
            return; /* 放弃重试 */
        } /* 失败分支结束 */
    } /* 创建分支结束 */
    ESP_LOGE(TAG, "adv start rc=%d, retry in %ums (%u/%u)", rc, delay_ms, s_adv_retry_cnt, ADV_RETRY_MAX); /* 显式报错 */
    esp_timer_start_once(s_adv_retry_timer, delay_ms * 1000); /* 启动一次性定时器 (微秒) */
}

static void start_adv(void) /* 启动 BLE 广播 (手机可发现并连接) */
{
    /* 厂商自定义数据 (AD type 0xFF): 公司ID 2B(0xFFFF=测试用) + 本机 IPv4 4B(网络序),
     * 手机扫描广播包即可读到本机 WiFi IP, 无需先连接; 未联网时 IP 全 0 */
    static uint8_t s_mfg[6] = { 0xFF, 0xFF, 0, 0, 0, 0 }; /* 厂商数据缓冲 (静态: 广播期间须一直有效) */
    memcpy(&s_mfg[2], &s_adv_ip, 4); /* 填入当前 IP (网络序原样拷贝) */

    struct ble_hs_adv_fields fields = { 0 }; /* 广播字段结构清零 */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP; /* 广播标志: 通用可发现 + 不支持传统 BR/EDR */
    fields.uuids128 = (ble_uuid128_t[]){ gatt_svc_uuid }; /* 广播携带 128 位服务 UUID */
    fields.num_uuids128 = 1; /* UUID 数量 */
    fields.mfg_data = s_mfg; /* 厂商数据指针 */
    fields.mfg_data_len = sizeof(s_mfg); /* 厂商数据长度 6B (总广播 29B, 未超 31B 上限) */
    int rc = ble_gap_adv_set_fields(&fields); /* 设置广播字段 */
    if (rc != 0) { /* 设置失败 */
        schedule_adv_retry(rc); /* 报错并安排重试 */
        return; /* 失败返回 */
    } /* 失败分支结束 */

    /* 设备名移入 Scan Response, 避免广播包超 31 字节上限 */
    struct ble_hs_adv_fields rsp_fields = { 0 }; /* 广播响应字段结构清零 */
    rsp_fields.name = (uint8_t *)s_dev_name; /* 响应字段: 设备名称 (含声道后缀, 如 7CH-RECEIVER-L) */
    rsp_fields.name_len = (uint8_t)strlen(s_dev_name); /* 设备名称长度 */
    rsp_fields.name_is_complete = 1; /* 标记为完整名称 */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields); /* 设置广播响应字段 */
    if (rc != 0) { /* 设置失败 */
        schedule_adv_retry(rc); /* 报错并安排重试 */
        return; /* 失败返回 */
    } /* 失败分支结束 */

    struct ble_gap_adv_params adv_params = { 0 }; /* 广播参数结构清零 */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   /* 可连接 */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   /* 可发现 */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(100); /* 广播间隔下限 100ms (160 单位) */
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(150); /* 广播间隔上限 150ms (240 单位) */
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, gap_event_handler, NULL); /* 启动广播 (公共地址, 无过滤, 事件回调 gap_event_handler; 时长必须传 BLE_HS_FOREVER, 传 0 会被当作 0ms 立即超时) */
    if (rc != 0) { /* 启动失败 */
        schedule_adv_retry(rc); /* 报错并安排带退避重试 (不再静默放弃) */
        return; /* 失败返回 */
    } /* 失败分支结束 */
    s_adv_retry_cnt = 0; /* 广播成功: 清零失败计数 */
    ESP_LOGI(TAG, "BLE advertising as '%s'", s_dev_name); /* 日志: 广播已开始 */
} /* 函数结束 */

/* ============ GAP 事件 ============ */
static int gap_event_handler(struct ble_gap_event *event, void *arg) /* GAP 事件回调: 处理连接/断开/广播结束 */
{ /* 函数体开始 */
    (void)arg; /* 忽略入参 */
    switch (event->type) { /* 按事件类型分发 */
    case BLE_GAP_EVENT_CONNECT: /* 连接事件 */
        if (event->connect.status == 0) { /* 连接成功 */
            g_conn_handle = event->connect.conn_handle; /* 记录连接句柄 */
            ESP_LOGI(TAG, "client connected, handle=%u", g_conn_handle); /* 日志输出句柄 */
            if (g_conn_cb) g_conn_cb(true); /* 通知调用方: BLE 已连接 */
        } else { /* 连接失败 */
            ESP_LOGE(TAG, "connect fail status=%d", event->connect.status); /* 日志: 连接失败原因 (调试用) */
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE; /* 清空连接句柄 */
            start_adv();                          /* 连接失败重新广播 */
        } /* 失败分支结束 */
        break; /* 退出 switch 分支 */
    case BLE_GAP_EVENT_DISCONNECT: /* 断开事件 */
        ESP_LOGI(TAG, "client disconnected, reason=%d", event->disconnect.reason); /* 日志输出断开原因 */
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE; /* 清空连接句柄 */
        g_status_val_handle = 0; /* 清空状态特征句柄 */
        if (g_conn_cb) g_conn_cb(false); /* 通知调用方: BLE 已断开 */
        start_adv();                              /* 断开后重新广播, 可再次配网 */
        break; /* 退出 switch 分支 */
    case BLE_GAP_EVENT_ADV_COMPLETE: /* 广播结束事件 */
        ESP_LOGW(TAG, "adv complete reason=%d", event->adv_complete.reason); /* 日志: 广播提前结束原因 (调试用) */
        start_adv(); /* 重新启动广播 */
        break; /* 退出 switch 分支 */
    default: /* 其他事件 */
        break; /* 忽略 */
    } /* switch 分支结束 */
    return 0; /* 返回成功 */
} /* 函数结束 */

/* ============ NimBLE host 回调 ============ */
static void on_sync(void) /* NimBLE 同步回调: 主机同步后初始化并广播 */
{ /* 函数体开始 */
    g_conn_handle = BLE_HS_CONN_HANDLE_NONE; /* 初始无连接 */
    uint8_t addr_type; /* 地址类型变量 */
    int rc = ble_hs_id_infer_auto(0, &addr_type); /* 自动推断本机地址类型 */
    if (rc != 0) { /* 推断失败 */
        ESP_LOGW(TAG, "infer addr rc=%d", rc); /* 日志告警 */
    } /* 失败分支结束 */
    rc = ble_svc_gap_device_name_set(s_dev_name); /* 设置 GAP 设备名称 (含声道后缀) */
    if (rc != 0) { /* 设置失败 */
        ESP_LOGW(TAG, "set name rc=%d", rc); /* 日志告警 */
    } /* 失败分支结束 */

    /* GATT 服务已注册, 缓存状态特征值句柄供 notify 使用 */
    uint16_t status_val_handle = 0; /* 局部句柄变量 (查特征值) */
    rc = ble_gatts_find_chr(&gatt_svc_uuid.u, &chr_prov_status_uuid.u, NULL, &status_val_handle); /* 查找状态特征的 value handle */
    if (rc == 0 && status_val_handle != 0) { /* 查找成功且句柄有效 */
        g_status_val_handle = status_val_handle; /* 保存全局状态特征句柄 */
        ESP_LOGI(TAG, "status char val_handle=%u", g_status_val_handle); /* 日志输出句柄值 */
    } else { /* 查找失败 */
        ESP_LOGW(TAG, "find status chr rc=%d", rc); /* 日志告警 */
    } /* 查找分支结束 */

    start_adv(); /* 同步后启动广播 */
} /* 函数结束 */

static void on_reset(int reason) /* NimBLE 重置回调: 主机异常重置时告警 */
{ /* 函数体开始 */
    ESP_LOGW(TAG, "nimble reset, reason=%d", reason); /* 日志输出重置原因 */
} /* 函数结束 */

/* ============ host 任务 ============ */
static void ble_host_task(void *param) /* BLE 主机任务: 运行 NimBLE 事件循环 */
{ /* 任务入口 */
    nimble_port_run(); /* 进入 NimBLE 主机事件循环 (阻塞) */
    nimble_port_freertos_deinit(); /* 事件循环退出后反初始化 */
    (void)param; /* 忽略入参 */
} /* 函数结束 */

/* ============ 对外入口 ============ */
void ble_provision_start(void) /* 对外接口: 启动 BLE 配网服务 (非阻塞, 可与 WiFi 共存) */
{ /* 函数体开始 */
    /* 配网模式: WiFi 未初始化, 只开 BLE; 工作模式: WiFi 已运行, BLE 常驻供随时改配置 */
    ESP_LOGI(TAG, "starting BLE provisioning service"); /* 日志: 启动 BLE 配网服务 */

    int rc = nimble_port_init(); /* 初始化 NimBLE 栈 */
    if (rc != ESP_OK) { /* 初始化失败 */
        ESP_LOGE(TAG, "nimble_port_init failed rc=%d", rc); /* 日志报错 */
        abort(); /* 终止运行 */
    } /* 失败分支结束 */

    ble_svc_gap_init(); /* 初始化 GAP 服务 */
    ble_svc_gatt_init(); /* 初始化 GATT 服务 */

    rc = ble_gatts_count_cfg(gatt_svcs); /* 统计 GATT 服务所需资源 */
    if (rc != 0) { /* 统计失败 */
        ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc); /* 日志报错 */
        abort(); /* 终止运行 */
    } /* 失败分支结束 */
    rc = ble_gatts_add_svcs(gatt_svcs); /* 注册 GATT 服务表 */
    if (rc != 0) { /* 注册失败 */
        ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc); /* 日志报错 */
        abort(); /* 终止运行 */
    } /* 失败分支结束 */

    ble_hs_cfg.sync_cb  = on_sync; /* 注册同步回调 */
    ble_hs_cfg.reset_cb = on_reset; /* 注册重置回调 */

    nimble_port_freertos_init(ble_host_task); /* 创建并启动 FreeRTOS 主机任务 */
    ESP_LOGI(TAG, "NimBLE host started, waiting for phone to provision"); /* 日志: 主机已启动, 等待手机配网 */
} /* 函数结束 */
