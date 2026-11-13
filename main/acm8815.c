/**
 * acm8815.c - 苏州至盛 ACM8815 / ACM8851 单声道 I2S 数字输入 D 类功放
 *             I2C 控制驱动 (ESP-IDF v6.x)
 *
 * 硬件连接 (见原理图 1-BF-L-0060-0.SchDoc):
 *   ESP32-C5 GPIO23 → Q5 (5551 NPN) Base → Q5 C → ACM8815 PDN (R28 1K 上拉 3V3)
 *     GPIO23=LOW  → Q5 OFF → PDN=HIGH (上拉) → 芯片上电工作
 *     GPIO23=HIGH → Q5 ON  → PDN=LOW  (导通) → 芯片 shutdown
 *
 *   ESP32-C5 GPIO24 → Q6 (5551 NPN) Base → Q6 C → ACM8815 MUTE (R33 1K 上拉 3V3)
 *     GPIO24=HIGH → Q6 ON  → MUTE pin=LOW  → ✅ 解除静音
 *     GPIO24=LOW  → Q6 OFF → MUTE pin=HIGH → ⚠️ 静音 (datasheet "high level active")
 *
 *   I2C: GPIO6=SDA (R31 4.7K 上拉), GPIO7=SCL (R30 4.7K 上拉)
 *   I2S: GPIO2=MCLK, GPIO3=BCLK, GPIO4=LRCLK/WS, GPIO5=SDIN
 *
 * 严格按原厂 ACM8815_export_new(1).c 五步序列:
 *   Step 1: Download m_reg_tab_initialization (cal)
 *   Step 2: Delay 10ms
 *   Step 3: Download m_reg_tab (main EQ/config)
 *   Step 4: Delay 100ms
 *   Step 5: Download m_reg_play_tab (进 Play)
 *
 * ⚠️ 原厂驱动注释: "BCLK must be running when writing play table"
 *   所以 Step 5 (play tab) 延后到 receiver watermark 到达、I2S 有数据后再调
 *
 * Datasheet 关键寄存器 (Page 24-27):
 *   0x04 STATE_CTRL: bits2-0=CTRL_STATE, 011=Play
 *     bit3 MUTE(软静音): 1=Mute, 0=Normal
 *     bit4 CH_HIZ: 1=HiZ
 *   0x07 I2S_FMT1: Reset=0x02 (16bit Philips)
 *     0x02=16bit Philips, 0x03=32bit Philips
 *   0x0F VOLUME: Reset=0xCF (~-103.5dB!) → 必须改写
 */

#include "acm8815.h"
#include "acm8815_regtable.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "acm8815"

#define REG_PAGE_SEL     0x00
#define REG_STATE_CTRL   0x04
#define REG_I2S_FMT1     0x07
#define REG_VOLUME       0x0F
#define REG_FAULT_RPT1   0x17
#define REG_FAULT_RPT2   0x18

#define PAGE_0 0x00

#define I2C_TIMEOUT_MS 100

/* ---------- 原厂 volumeTable (17 档, page4 reg0x40~0x43) ----------
 * index 0=0dB(最响), index 16=-109.5dB(几乎静音)
 * 每档 4 字节, 直接写入 page4 的 0x40~0x43 */
static const uint8_t VOL_TABLE[17][4] = {
    {0x00,0x80,0x00,0x00}, /*  0= 0.0dB  */
    {0x00,0x5f,0xfc,0x89}, /*  1=-2.5dB  */
    {0x00,0x43,0xf4,0x05}, /*  2=-5.5dB  ← ~53% 线性幅度, 最接近 50% */
    {0x00,0x32,0xf5,0x2d}, /*  3=-8.0dB  */
    {0x00,0x26,0x36,0x80}, /*  4=-10.5dB */
    {0x00,0x1b,0x0d,0x7b}, /*  5=-13.5dB */
    {0x00,0x14,0x49,0x61}, /*  6=-16.0dB */
    {0x00,0x0f,0x36,0x7c}, /*  7=-18.5dB */
    {0x00,0x0a,0xc5,0x15}, /*  8=-21.5dB */
    {0x00,0x08,0x13,0x85}, /*  9=-24.0dB */
    {0x00,0x06,0x0e,0x6c}, /* 10=-26.5dB */
    {0x00,0x03,0xd2,0x40}, /* 11=-30.5dB */
    {0x00,0x02,0x69,0x3c}, /* 12=-34.5dB */
    {0x00,0x00,0xc3,0x30}, /* 13=-44.5dB */
    {0x00,0x00,0x4d,0xb5}, /* 14=-52.5dB */
    {0x00,0x00,0x26,0xf2}, /* 15=-58.5dB */
    {0x00,0x00,0x00,0x1c}, /* 16=-109.5dB */
};
static const float VOL_TABLE_DB[17] = {
     0.0f, -2.5f, -5.5f, -8.0f, -10.5f, -13.5f, -16.0f, -18.5f,
    -21.5f, -24.0f, -26.5f, -30.5f, -34.5f, -44.5f, -52.5f, -58.5f, -109.5f
};

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool                    s_present;
static uint8_t                 s_cur_page;

/* ---------- 底层 I2C ---------- */
static esp_err_t acm8815_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t acm8815_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
}

static esp_err_t acm8815_set_page(uint8_t page)
{
    if (s_cur_page == page) return ESP_OK;
    esp_err_t err = acm8815_write_reg(REG_PAGE_SEL, page);
    if (err == ESP_OK) s_cur_page = page;
    return err;
}

/* ---------- 整表写入 ----------
 * 每对寄存器一次独立 I2C 事务 (防止 PAGE_SELECT 被中间长事务吞掉) */
static esp_err_t acm8815_write_burst(const uint8_t *buf, int pairs)
{
    int fails = 0;
    for (int i = 0; i < pairs; i++) {
        uint8_t reg = buf[i * 2], val = buf[i * 2 + 1];
        esp_err_t e = acm8815_write_reg(reg, val);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "  pair[%d] 0x%02X=0x%02X FAIL (%s)",
                     i, reg, val, esp_err_to_name(e));
            fails++;
        }
        if (reg == REG_PAGE_SEL) s_cur_page = val;
    }
    if (fails) {
        ESP_LOGE(TAG, "  table: %d/%d pairs failed", fails, pairs);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ---------- 初始化: 严格按原厂五步 ---------- */
esp_err_t acm8815_init(void)
{
    /* ============================================================
     * Phase 0: PDN 时序 (datasheet Page 21-22)
     *   电源稳定 → PDN LOW(shutdown) → ≥1ms → PDN HIGH → 等 5ms → I2C
     *   GPIO23=HIGH→Q5 on→PDN=LOW; GPIO23=LOW→Q5 off→PDN=HIGH
     * ============================================================ */
#if ACM8815_PDN_GPIO >= 0
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << ACM8815_PDN_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
    }
    /* 先强制 shutdown, 干净复位 */
    ESP_LOGI(TAG, "PDN: force shutdown (GPIO%d=HIGH)", ACM8815_PDN_GPIO);
    gpio_set_level(ACM8815_PDN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
    /* 上电 */
    ESP_LOGI(TAG, "PDN: power-up (GPIO%d=LOW → PDN=HIGH)", ACM8815_PDN_GPIO);
    gpio_set_level(ACM8815_PDN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  /* datasheet ≥5ms 让芯片 start-up */
#endif

    /* ============================================================
     * MUTE: GPIO24=HIGH → Q6 on → MUTE pin=LOW → 解除静音
     * ============================================================ */
#if ACM8815_MUTE_GPIO >= 0
    {
        gpio_config_t mi = {
            .pin_bit_mask = 1ULL << ACM8815_MUTE_GPIO,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&mi);
        gpio_set_level(ACM8815_MUTE_GPIO, 1);
        ESP_LOGI(TAG, "MUTE: GPIO%d=HIGH → 解除静音 (回读=%d)",
                 ACM8815_MUTE_GPIO, gpio_get_level(ACM8815_MUTE_GPIO));
    }
#endif

    /* ============================================================
     * I2C 初始化 + 探测
     * ============================================================ */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = ACM8815_I2C_SDA_GPIO,
        .scl_io_num        = ACM8815_I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "i2c_new_master_bus failed");

    uint8_t addr = 0;
    bool found = false;
    for (uint8_t a = 0x40; a <= 0x47; a++) {
        if (i2c_master_probe(s_bus, a, I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "[SCAN] ACK at 0x%02X", a);
            addr = a; found = true;
            break;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "ACM8815 not responding at 0x40~0x47!");
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = ACM8815_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "add device failed");
    s_cur_page = 0xff;
    s_present = true;
    ESP_LOGI(TAG, "I2C addr=0x%02X OK", addr);

    /* ============================================================
     * 原厂五步 init (严格对齐 ACM8815_export_new(1).c)
     * ============================================================ */

    /* Step 1: m_reg_tab_initialization (calibration) */
    ESP_LOGI(TAG, "[1/5] init tab (%d pairs)", ACM_CAL_TAB_N);
    esp_err_t e = acm8815_write_burst(acm_cal_tab, ACM_CAL_TAB_N);
    if (e != ESP_OK) { ESP_LOGE(TAG, "Step 1 failed"); return e; }

    /* Step 2: Delay 10ms */
    ESP_LOGI(TAG, "[2/5] delay 10ms");
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Step 3: m_reg_tab (main EQ/config) */
    ESP_LOGI(TAG, "[3/5] main tab (%d pairs)", ACM_CFG_TAB_N);
    e = acm8815_write_burst(acm_cfg_tab, ACM_CFG_TAB_N);
    if (e != ESP_OK) { ESP_LOGE(TAG, "Step 3 failed"); return e; }

    /* Step 4: Delay 100ms */
    ESP_LOGI(TAG, "[4/5] delay 100ms");
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Step 5: m_reg_play_tab — 严格对齐原厂, init 里连续执行五步 */
    ESP_LOGI(TAG, "[5/5] play tab (%d pairs)", ACM_PLAY_TAB_N);
    e = acm8815_write_burst(acm_play_tab, ACM_PLAY_TAB_N);
    if (e != ESP_OK) { ESP_LOGE(TAG, "Step 5 failed"); return e; }

    /* Step 5 后: 显式置 I2S_FMT1 = 0x03 (32bit Philips)
     * 芯片复位默认 0x02 (16bit), 而 I2S 侧是 64fs / 32bit 槽.
     * play tab 不含 0x07, 若 Play 延后或失败, 芯片会按 16bit 解析 32bit 槽,
     * 表现为声音变噪声/卡顿, 所以 init 阶段就先写一次并回读确认;
     * acm8815_play() 里还会再校验一次, 防止 play tab 覆盖 */
    {
        uint8_t fmt = 0;
        s_cur_page = 0xff;
        acm8815_set_page(PAGE_0);
        acm8815_write_reg(REG_I2S_FMT1, 0x03);
        acm8815_read_reg(REG_I2S_FMT1, &fmt);
        if (fmt != 0x03) {
            ESP_LOGW(TAG, "I2S_FMT1=0x%02X (expect 0x03) write failed", fmt);
        } else {
            ESP_LOGI(TAG, "I2S_FMT1=0x03 (32bit Philips) ✅");
        }
    }

    /* Step 5 后: 用原厂 page4 volumeTable 设 50% 音量 (-5.5dB) */
    acm8815_set_volume_db(-6.0f);

    ESP_LOGI(TAG, "init OK ✅ (五步全部完成)");
    return ESP_OK;
}

/* ---------- Step 5: 进 Play (原厂 play tab) ---------- */
esp_err_t acm8815_play(void)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;

    /* 原厂 play tab */
    ESP_LOGI(TAG, "=== PLAY TAB: %d pairs ===", ACM_PLAY_TAB_N);
    esp_err_t e = acm8815_write_burst(acm_play_tab, ACM_PLAY_TAB_N);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "play tab failed: %s", esp_err_to_name(e));
        return e;
    }

    /* 回读 STATE 确认 */
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        uint8_t st = 0;
        s_cur_page = 0xff;
        acm8815_set_page(PAGE_0);
        acm8815_read_reg(REG_STATE_CTRL, &st);
        ESP_LOGI(TAG, "STATE=0x%02X (expect 0x03=Play) %s",
                 st, (st == 0x03) ? "✅" : "❌ MISMATCH");

        /* 如果 play tab 里 I2S_FMT 被覆盖, 重新写 32bit */
        uint8_t fmt = 0;
        s_cur_page = 0xff;
        acm8815_set_page(PAGE_0);
        acm8815_read_reg(REG_I2S_FMT1, &fmt);
        if (fmt != 0x03) {
            ESP_LOGW(TAG, "play tab 覆盖了 I2S_FMT=0x%02X, 重写 0x03", fmt);
            acm8815_write_reg(REG_I2S_FMT1, 0x03);
        }
    }

    ESP_LOGI(TAG, "Play done ✅");
    return ESP_OK;
}

esp_err_t acm8815_set_hw_mute(bool mute)
{
#if ACM8815_MUTE_GPIO >= 0
    gpio_set_level(ACM8815_MUTE_GPIO, mute ? 0 : 1);
    return ESP_OK;
#else
    (void)mute;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t acm8815_set_sw_mute(bool mute)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    uint8_t cur = 0;
    s_cur_page = 0xff;
    acm8815_set_page(PAGE_0);
    acm8815_read_reg(REG_STATE_CTRL, &cur);
    uint8_t val = mute ? (cur | 0x08) : (cur & ~0x08);
    return acm8815_write_reg(REG_STATE_CTRL, val);
}

esp_err_t acm8815_set_volume(uint8_t atten_db_index)
{
    if (!s_present) return ESP_ERR_INVALID_STATE;
    if (atten_db_index > 16) atten_db_index = 16;

    /* 原厂推荐方式: page4 reg0x40~0x43 */
    esp_err_t e = acm8815_set_page(4);
    if (e != ESP_OK) return e;
    for (int i = 0; i < 4; i++) {
        e = acm8815_write_reg(0x40 + i, VOL_TABLE[atten_db_index][i]);
        if (e != ESP_OK) return e;
    }
    ESP_LOGI(TAG, "volume: page4 index=%d (%.1fdB)", atten_db_index, VOL_TABLE_DB[atten_db_index]);
    return ESP_OK;
}

esp_err_t acm8815_set_volume_db(float db)
{
    if (db > 0.0f)  db = 0.0f;
    if (db < -109.5f) db = -109.5f;

    /* 找最近档 */
    int best = 0;
    float best_diff = 1e9f;
    for (int i = 0; i < 17; i++) {
        float diff = db - VOL_TABLE_DB[i];
        if (diff < 0) diff = -diff;
        if (diff < best_diff) { best_diff = diff; best = i; }
    }
    return acm8815_set_volume(best);
}

bool acm8815_present(void) { return s_present; }
bool acm8815_is_present(void) { return s_present; }
