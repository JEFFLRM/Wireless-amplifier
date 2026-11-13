/**
 * acm8815.h - 苏州至盛 ACM8815 / ACM8816 / ACM86xx 系列 单声道 I2S 数字输入 D 类功放驱动
 *
 * 芯片特点:
 *   - 内置 DSP + 集成 MOSFET 的 200W 单声道 D 类功放
 *   - I2S 数字音频直入 (内部 PLL 以 BCLK 为参考时钟, 无需 MCLK)
 *   - 喇叭直接接功放输出级 (取代 ES9018K2M DAC + 模拟功放两级方案)
 *
 * ⚠️ 关键差异 (与 ES9018K2M / 简化版驱动):
 *   1. I2C 是多页架构: reg 0x00 为 PAGE_SELECT, 写值决定后续寄存器页 (0/1/4/6/7...)
 *      每次操作寄存器前必须先切页 acm8815_set_page(page).
 *   2. 上电默认 CTRL_STATE = Digital Off (无声), 必须 I2C 配置后才出声.
 *   3. 原厂要求严格 5 步初始化序列, 不能跳过任何一步 (见 acm8815.c 注释).
 *
 * 寄存器架构 (PAGE_SELECT = reg 0x00):
 *   Page 0: STATE_CTRL(0x04), AMP_CTRL1/2(0x01/0x02), I2S_FMT1(0x07), CLASS_D(0x03/05/06/11), VOLUME(0x0F), CAL(0xFC/FD/FE), STATE/FAULT/WARN(0x16~0x19)
 *   Page 1: Class D 电源/偏置寄存器 (reg 0x01, 0x11...)
 *   Page 4: 音量表 volumeTable 四字节 (reg 0x40~0x43), 原厂推荐方式
 *   Page 6/7: EQ 滤波器系数 (驱动按原厂默认 All-Pass 直通, 不写)
 *
 * Page 0 关键寄存器值 (依据 ACM8815_export_new.c 原厂参考驱动):
 *   0x01 AMP_CTRL1   : FAULT_CLR(bit7) + bit2 = 0x84 (原厂 init 强制值, 不能漏 bit2!)
 *   0x02 AMP_CTRL2   : 原厂 init 写 0x09
 *   0x03 D_CTRL      : 0x40
 *   0x04 STATE_CTRL  : 0x00=Off; 0x03=Play 出声; 0x0b=Play+SoftMute (HPD workaround 过渡态); 0x0e=静音
 *   0x05 D_CTRL      : 0xcc
 *   0x06 D_CTRL      : 0x71
 *   0x07 I2S_FMT1    : 0x03 = I2S 标准 + 32bit (本工程 BCLK=64fs 32bit 槽)
 *   0x11 D_CTRL      : 0xc0=默认调制; 0xc8=OCP workaround 临时值 (Play Script 用到)
 *   0xFC/FD/FE CAL   : 原厂 init 强制 0x86 / 0x15 / 0x25 (校准寄存器)
 *   0x16~0x19        : STATE/FAULT/WARN 只读报告
 *
 * I2C 地址: 7bit = 0x40 + (ADR 电阻选择的 2bit); 4.7k=0x40 / 15k=0x41 / 47k=0x42 / 120k=0x43
 *           驱动自动探测 0x40~0x43.
 *
 * PDN / MUTE 硬件 (板上 NPN 5551 反相驱动, ESP32→三极管→ACM8815, 电平反转):
 *   - PDN_CTRL (GPIO23):  ESP32=低(0) → Q5 截止 → PDN 被上拉高 → 芯片上电
 *                         ESP32=高(1) → Q5 导通  → PDN 拉低   → 芯片掉电
 *   - MUTE_CTRL (GPIO24): ESP32=高(1) → Q6 导通  → MUTE 拉低   → 解除硬件静音
 *                         ESP32=低(0) → Q6 截止  → MUTE 被上拉高 → 硬件静音
 *   两个 GPIO 均无需外部上拉, 板载三极管侧已有. 代码内部已做反相处理.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* ---------- 硬件引脚 (按实际接线修改) ---------- */
#define ACM8815_I2C_SDA_GPIO  GPIO_NUM_6     /* I2C SDA */
#define ACM8815_I2C_SCL_GPIO  GPIO_NUM_7     /* I2C SCL */
#define ACM8815_I2C_FREQ_HZ   100000         /* 100kHz: 原厂整表一次灌入 (700+ 字节),
                                              * 10kHz 会超过传输超时且时序不匹配 */
#define ACM8815_I2C_ADDR_BASE 0x44           /* 实测本板 = 0x45 (基址 0x44 + ADR=1, ACM8851) */
#define ACM8815_I2C_ADDR_MAX  0x47           /* ADR 四档 0x44~0x47 */
#define ACM8815_PDN_GPIO      GPIO_NUM_24    /* PDN_CTRL (NPN 反相, 内部已处理) */
#define ACM8815_MUTE_GPIO     GPIO_NUM_23    /* MUTE_CTRL (NPN 反相, 内部已处理; -1=仅 I2C 软静音) */

/* ---------- 默认音量 (page4 volumeTable Index) ----------
 * 原厂 volumeTable 17 档: Index 0=0dB, 1=-2.5dB, 2=-5.5dB, 3=-8.0dB, 4=-10.5dB,
 *                         Index 5=-13.5dB (≈21% 线性幅度, 对应 DEFAULT 20%),
 *                         ... Index 16=-109.5dB (几乎静音)
 */
#define ACM8815_DEFAULT_VOL_DB_INDEX  5      /* -13.5dB ≈ 20% 线性幅度 */

/**
 * @brief 完整初始化流程 (参照 ACM8815_export_new.c 原厂参考驱动):
 *   Phase 1: PDN/MUTE GPIO (NPN 反相) → 上电 + 解除静音
 *   Phase 2: I2C 总线 + 设备自动探测
 *   Phase 3: Init Script (校准寄存器, page0) + Delay 10ms
 *   Phase 4: Class D 控制 (page1+page0 7 行) + I2S 32bit (page0 reg0x07=0x03, 回读校验)
 *   Phase 5: Delay 100ms (DSP 参数生效)
 *   Phase 6: Play Script (含 HPD/OCP startup workaround, 最终 page0 reg0x04=0x03)
 *   Phase 7: 默认音量 (page4 volumeTable Index 5 = -13.5dB ≈ 20%)
 *   Phase 8: 诊断日志 (读 STATE/FAULT/WARN 寄存器)
 * @return ESP_OK 成功; ESP_ERR_NOT_FOUND I2C 探测全失败 (PDN 可能仍低); 其他为 I2C 错误
 */
esp_err_t acm8815_init(void);

/**
 * @brief 设置音量 (原厂推荐方式: page4 volumeTable 四字节)
 * @param atten_db_index volumeTable 索引 0~16:
 *   0=0dB(最响), 5=-13.5dB(20%), 16≈-109.5dB(几乎静音); 越界自动钳位
 */
esp_err_t acm8815_set_volume(uint8_t atten_db_index);

/**
 * @brief 设置音量 (dB 便捷接口, 自动找最近档位)
 * @param db 目标 dB 值, 范围 [-109.5, 0.0]; 如 -13.5 或 -14.0
 */
esp_err_t acm8815_set_volume_db(float db);

/**
 * @brief I2C 软静音 (page0 reg0x04: 0x0e=静音 / 0x03=Play 出声, 音量寄存器不变)
 * @param mute true=静音 / false=取消静音
 */
esp_err_t acm8815_set_mute(bool mute);

/**
 * @brief 硬件 PDN 控制 (板上 NPN 反相, 内部自动反转; 若 GPIO==-1 返回 NOT_SUPPORTED)
 * @param on true=上电 / false=掉电
 * @note 掉电后芯片 I2C 不可达, 下次使用需重新 acm8815_init()
 */
esp_err_t acm8815_set_pdn(bool on);

/**
 * @brief 硬件 MUTE 脚控制 (板上 NPN 反相, 内部自动反转; 若 GPIO==-1 返回 NOT_SUPPORTED)
 * @param mute true=硬件静音 / false=解除硬件静音
 */
esp_err_t acm8815_set_hw_mute(bool mute);

/** @brief 查询芯片是否已初始化成功 (false = 无声态) */
bool acm8815_present(void);

/**
 * @brief 进入 Play 状态 (含 HPD/OCP workaround)
 * @note 必须在 I2S 有持续数据流时调用! 否则芯片检测不到时钟会自动复位
 *       建议在 RTP watermark 到达、开始播放时调用
 */
esp_err_t acm8815_play(void);
