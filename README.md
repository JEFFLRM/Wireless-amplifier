---
AIGC:
    Label: "1"
    ContentProducer: 001191440300708461136T1XGW3
    ProduceID: e64b5553ca31f0318c1e91126164a919_5079813ec79911f1b5a6525400393706
    ReservedCode1: tq2ampz0QCm/ijkoaQVuYe63NX/Glx49qPmglcbG+4eTJP45jwdAbRy2xSsH0Ha8cqEo19oDHRN5KXSMJMsy8nKscFeqsq/+1c0ZTJNSY/Ym5ZUFNFTV3n2l7TbYRpb+WIgCmwAQv9IWzc4g8/w4hXswQ6m5E0UdA6ZMk89S7tXlgM9nO848K2PCh14=
    ContentPropagator: 001191440300708461136T1XGW3
    PropagateID: e64b5553ca31f0318c1e91126164a919_5079813ec79911f1b5a6525400393706
    ReservedCode2: tq2ampz0QCm/ijkoaQVuYe63NX/Glx49qPmglcbG+4eTJP45jwdAbRy2xSsH0Ha8cqEo19oDHRN5KXSMJMsy8nKscFeqsq/+1c0ZTJNSY/Ym5ZUFNFTV3n2l7TbYRpb+WIgCmwAQv9IWzc4g8/w4hXswQ6m5E0UdA6ZMk89S7tXlgM9nO848K2PCh14=
---



# ESP32-HIFI 接收端 (ESP32-C5)

7.1CH 分布式无线音频接收端固件：5GHz WiFi STA 加入组播组 `239.100.7.1:5004` 接收 RTP 组播音频，
XOR FEC（N=10）丢包恢复，按 `slot_index` 提取本机单声道，I2S 输出 96kHz/24bit 到 PCM5102A
（single 模式），或 TDM 8 声道输出（full 模式）。

与发送端 `HIFI/main/sender.c`（当前为 W5500 以太网发送）协议严格对齐：RTP/FEC 字节布局与承载介质
无关，字节一致。

## 工程结构

```
D:\jeff\资料\ESP32_HIFI\
├── CMakeLists.txt          # 工程级 (project RECEIVER_HIFI)
├── sdkconfig.defaults      # ESP32-C5 / 5GHz WiFi / NimBLE
├── main\
│   ├── CMakeLists.txt      # 组件级 (ble_provision.c + receiver.c)
│   ├── receiver.c          # 核心: WiFi STA -> 组播 -> RTP/FEC -> I2S 输出
│   ├── ble_provision.c     # NimBLE GATT 配网 + NVS 持久化
│   └── ble_provision.h     # 配网配置结构 (含 slot_index / output_mode)
└── README.md
```

## 一、接线表

### 1. I2S 输出（PCM5102A）

| ESP32-C5 GPIO | 信号 | PCM5102A 引脚 | 说明 |
|---|---|---|---|
| GPIO2 | MCLK | SCK | 主时钟 256fs = 24.576MHz，PCM5102A 内部 PLL 也可锁定 BCLK，可不接 |
| GPIO3 | BCLK | BCK | 位时钟，single: 3.072MHz / full: 24.576MHz |
| GPIO4 | WS | LRCK | 声道时钟（帧同步）96kHz |
| GPIO5 | DOUT | DIN | 音频数据 24bit |
| - | - | FMT | 接地（I2S 格式，Philips 标准） |
| - | - | DEMP / FLT / XSMT | 按 PCM5102A 手册设置（通常 FLT 悬空=低延迟） |
| 3V3 | VDD / VIN | 3.3V | 电源（DVDD/AVDD，如需 1.8V 模式短接 VDD 至 1.8V） |
| GND | GND | GND | 共地 |
| 3V3 | - | VREF | 内部基准，接 0.1uF 退耦电容到 GND |

> single 模式：`I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(24BIT, MONO)`，MONO 下硬件将单声道数据复制到左右声道。
> full 模式：TDM 8 slot × 24bit，slot0..7 对应 L/R/C/LFE/SL/SR/SBL/SBR。

### 2. 配网 / 控制

| ESP32-C5 GPIO | 功能 | 说明 |
|---|---|---|
| GPIO9 | BOOT 键 | 低电平按下；长按 3 秒清空 NVS 配置并重启进入 BLE 配网 |
| - | 手机 BLE | 广播名 `7CH-RECEIVER`，GATT Service `0000ff00-...` |

### 3. 网络拓扑

- 发送端：W5500 SPI 以太网（GPIO10/11/12/13/14/15），组播出口绑定 eth，TTL 默认 1（可调大）。
- 接收端：ESP32-C5 WiFi STA **5GHz** 连接路由器，`IP_ADD_MEMBERSHIP` 加入 `239.100.7.1:5004`。
- **路由器要求**：需支持 IGMP Snooping / 组播转发到无线（多数家用路由默认组播转发到 2.4G/5G 无线，
  若有线发无线组播丢包，请调高发送端 `DEF_MCAST_TTL` 至 2~4，并检查路由器"组播增强"选项）。

## 二、时钟关系（96kHz）

| 参数 | single（标准 I2S MONO） | full（TDM 8 slot） |
|---|---|---|
| 采样率 fs | 96 kHz | 96 kHz |
| 数据位宽 | 24 bit | 24 bit |
| slot 数 | 1（MONO 硬件复制 L/R） | 8 |
| BCLK | 1 × 32bit × 96k = **3.072 MHz** | 8 × 32bit × 96k = **24.576 MHz** |
| MCLK | 256fs = **24.576 MHz** | 256fs = **24.576 MHz** |
| WS/LRCK | 96 kHz | 96 kHz |
| 帧数据 | 每 1ms 一个 RTP 包（96 帧 × 8 声道 × 3B = 2304B） | 同左 |

发送端时钟树（供联调参考）：CS5368 从模式，MCLK/SCLK = 256fs = 24.576MHz；I2S0 TDM RX 8slot×32bit。

## 三、8 台接收端 slot_index 配置表

声道映射：`slot0..7 = L/R/C/LFE/SL/SR/SBL/SBR`。

| 接收端编号 | 声道 | slot_index | output_mode | 建议摆放 |
|---|---|---|---|---|
| RX1 | 左前置 L | 0 | single | 左前 |
| RX2 | 右前置 R | 1 | single | 右前 |
| RX3 | 中置 C | 2 | single | 中央 |
| RX4 | 低音 LFE | 3 | single | 低音炮位 |
| RX5 | 左环绕 SL | 4 | single | 左侧 |
| RX6 | 右环绕 SR | 5 | single | 右侧 |
| RX7 | 左后环绕 SBL | 6 | single | 左后 |
| RX8 | 右后环绕 SBR | 7 | single | 右后 |

> full 模式：单台设备配置 `output_mode="full"` 时输出全部 8 声道（接多声道 DAC 时使用），此时
> slot_index 可留默认 0（不影响 full 输出）。

## 四、BLE 配网 JSON

上电无有效配置时进入 BLE 配网（关 WiFi 开 BLE），手机连接 `7CH-RECEIVER`，向
Char2（`0000ff02-...`）写入：

```json
{
  "mode": "sta",
  "ssid": "MY-5G-ROUTER",
  "password": "wifi-password",
  "channel": 0,
  "mcast_ip": "239.100.7.1",
  "mcast_port": 5004,
  "slot_index": 0,
  "output_mode": "single"
}
```

- `mode`：`"sta"` 连路由器（推荐）；`"ap"` 连 5GHz SoftAP（兼容直连发送端热点场景）。
- 缺省字段自动补全：`mcast_ip=239.100.7.1`、`mcast_port=5004`、`slot_index=0`、`output_mode="single"`。
- 写入成功 Char3 通知 `ok`，1 秒后重启进入工作模式。
- 任意时刻长按 BOOT 键 3 秒：清配置重启，重新进入 BLE 配网。

## 五、烧录与联调步骤

### 1. 环境与构建

- ESP-IDF **v5.4+**，目标芯片 `esp32c5`（ESP-IDF 需支持 ESP32-C5；5GHz WiFi 依赖
  `CONFIG_ESP_WIFI_5G_SUPPORT=y`，已写入 sdkconfig.defaults）。
- 用 VSCode + ESP-IDF 插件打开本目录（**含 CMakeLists.txt 的工程根目录**，即 `D:\jeff\资料\ESP32_HIFI`；
  若 VSCode 对中文路径构建异常，复制到纯英文路径如 `D:\jeff\esp32_hifi\RECEIVER_NEW` 再打开）。
- 点击 Build（或 `idf.py build`）。

### 2. 烧录

```bash
idf.py -p COMx flash monitor
```

推荐 `idf.py flash monitor` 观察日志：上电后应看到 `BLE advertising as '7CH-RECEIVER'`（无配置时）。

### 3. 配网

1. 手机 BLE 扫描到 `7CH-RECEIVER`，连接。
2. 向 Char2 写入上述 JSON（可用 nRF Connect / 自写小程序）。
3. Char3 收到 `ok` 后设备重启。
4. 重启日志应显示 `config: mode=sta ssid=... slot=0 out=single mcast=239.100.7.1:5004`，
   随后 `STA connected to '...'`、`joined multicast 239.100.7.1:5004`。

### 4. 联调（与发送端）

1. 发送端（W5500 以太网）上电：DHCP 获取 IP，日志 `multicast 239.100.7.1:5004 ready`。
2. 确认 PC/路由器能 ping 通发送端与接收端；组播可达（路由器组播转发开启）。
3. 接收端 5 秒周期打印统计：
   `stat: rx=1000 fec=100 rec=0 miss=0 drop=0 under=0`
   - `rec` = FEC 恢复成功包数，`miss` = 未能恢复静音帧数；两者持续为 0 表示无损。
4. 听音：single 模式 PCM5102A 应有对应声道声音；切换 slot_index 验证声道对应关系。
5. 丢包验证：发送端调大 `DEF_MCAST_TTL` 或人为制造弱网，观察 `rec/miss` 变化，确认 FEC 生效。

## 五·补充、故障排查：扫描不到 7CH-RECEIVER（BLE 配网服务未开启）

现象：PC/手机 BLE 扫描不到接收端，无法连接配网。

排查步骤：

1. **检查是否有残留配置**：接收端只要 NVS 中存在有效配置（命名空间 `prov`，键 `cfg`），
   上电会直接进入工作模式（STA，不开 BLE），表现为"蓝牙服务未开启"。
   串口日志会打印：
   ```
   W (…) receiver: existing valid config detected, entering work mode (BLE off); hold BOOT 3s to clear config and re-enter provisioning
   ```
2. **长按 BOOT 键 3 秒**：清空 NVS 配置并重启，强制重新进入 BLE 配网。
   重启后日志应显示 `BLE advertising as '7CH-RECEIVER'`。
3. **若仍无广播**：检查广播启动是否失败（自 v1.0.1 起失败会自动带退避重试
   1s/2s/4s/8s…封顶 10s，最多 8 次后重启重试，串口会有 `E (…) ble_prov: adv start rc=...`）：
   - 确认 `CONFIG_BT_ENABLED=y`、`CONFIG_BT_NIMBLE_ENABLED=y`、
     `CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y` 已启用（sdkconfig.defaults 已默认包含）；
   - 确认未在配网模式下同时初始化 WiFi（配网模式只开 BLE）。
4. **验证流程**：上电日志依次应看到
   `no valid config, entering BLE provisioning...` →
   `NimBLE host started, waiting for phone to provision` →
   `BLE advertising as '7CH-RECEIVER'`，随后即可被扫描连接。

## 六、与 sender.c 协议核对点

| 项 | sender.c | receiver.c | 一致 |
|---|---|---|---|
| RTP 头 | packed 12B: v_p_x_cc=0x80, pt, seq/ts/ssrc 网络序, ssrc=0x7C41 | 同左 | ✅ |
| Payload | 96帧 × 8ch × 3B = 2304B，帧间交错，24bit 小端紧凑 | 同左（extract_slot 3B 步进） | ✅ |
| FEC | 每 10 数据包 XOR 1 校验包；FEC 包 seq = 本组首包 seq - 1 | 恢复 [F+1, F+10] 窗口 | ✅ |
| TS | 每数据包 +96（FEC 包不加） | 接收端不依赖 ts，按 seq 排序 | ✅ |
| 声道映射 | slot0..7 = L/R/C/LFE/SL/SR/SBL/SBR | 同左 | ✅ |

## 七、编译注意事项

1. **5GHz 依赖**：`CONFIG_ESP_WIFI_5G_SUPPORT=y`、`CONFIG_ESP_WIFI_ENABLED=y` 必须保留；
   ESP32-C5 需 ESP-IDF 支持版本（v5.4+ 主线）。
2. **BLE/WiFi 异频**：配网模式仅开 BLE（2.4GHz），工作模式仅开 WiFi（5GHz），sdkconfig 中
   NimBLE 仅保留 peripheral 角色；不要在配网模式下初始化 WiFi。
3. **单核**：ESP32-C5 单核，所有任务 `xTaskCreatePinnedToCore(..., 0)`；网络任务优先级 5 > I2S 4，
   保证收包不被播放阻塞。
4. **I2S 驱动**：24bit 走 3 字节紧凑搬运（`I2S_DATA_BIT_WIDTH_24BIT` 原生布局），不要在 ring 中做
   32bit 对齐，否则 BCLK/数据错位。
5. **内存**：`RING_PKTS=16 × 2304B ≈ 36KB` + FEC 缓存 `20 × 2304B ≈ 45KB` 均堆分配；
   full 模式单包 2304B，网络任务栈 8192 字节足够，勿再开大栈。
6. **STA 连接超时**：15s 未连上仅打印错误不阻塞，socket 仍会 bind/join，便于排查组播侧问题。
7. **中文路径**：本工程路径含中文，若 VSCode/ESP-IDF 构建报路径错误，请复制到纯英文目录再构建
   （代码本身 UTF-8 无影响）。
*（内容由AI生成，仅供参考）*
