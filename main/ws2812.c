/**
 * ws2812.c - WS2812B 单总线 RGB LED 驱动实现 (ESP32-C5, RMT TX)
 *
 * 时序 (us): T0H=0.3 T0L=0.9 / T1H=0.9 T1L=0.3;
 * RMT 分辨率 10MHz (1 tick = 0.1us) → 0.3us = 3 tick, 0.9us = 9 tick。
 * 数据布局: 每灯珠 3 字节, 顺序 G R B, 每字节 MSB first。
 */
#include "ws2812.h" /* 本模块头文件 */
#include "driver/rmt_tx.h" /* RMT TX 驱动 (通道/发送) */
#include "driver/rmt_encoder.h" /* RMT 编码器 (bytes encoder) */
#include "esp_log.h" /* ESP 日志 */
#include "esp_err.h" /* esp_err_to_name */

#define TAG "ws2812" /* 日志标签 */

#define WS2812_RESOLUTION_HZ  10000000 /* RMT 分辨率 10MHz (0.1us/tick) */

static rmt_channel_handle_t  s_chan    = NULL; /* RMT TX 通道句柄 */
static rmt_encoder_handle_t  s_encoder = NULL; /* 字节编码器句柄 */

esp_err_t ws2812_init(int gpio_num) /* 初始化驱动: 建 RMT 通道 + bytes 编码器 */
{ /* 函数体开始 */
    if (s_chan != NULL) { /* 已初始化过, 幂等返回 */
        return ESP_OK; /* 直接成功 */
    } /* 已初始化分支结束 */

    rmt_tx_channel_config_t tx_cfg = { /* RMT TX 通道配置 */
        .gpio_num          = (gpio_num_t)gpio_num, /* 数据脚 */
        .clk_src           = RMT_CLK_SRC_DEFAULT, /* 默认时钟源 */
        .resolution_hz     = WS2812_RESOLUTION_HZ, /* 10MHz 分辨率 */
        .mem_block_symbols = 64, /* 内存块符号数 (单灯珠 24 符号, 64 足够) */
        .trans_queue_depth = 4, /* 发送队列深度 */
    }; /* 通道配置结束 */
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_chan); /* 创建 TX 通道 */
    if (err != ESP_OK) { /* 创建失败 */
        ESP_LOGE(TAG, "new tx channel failed: %s", esp_err_to_name(err)); /* 日志报错 */
        return err; /* 返回错误 */
    } /* 失败分支结束 */

    rmt_bytes_encoder_config_t enc_cfg = { /* 字节→RMT 符号编码配置 (WS2812 时序) */
        .bit0  = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 }, /* BIT0: 高 0.3us + 低 0.9us */
        .bit1  = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 }, /* BIT1: 高 0.9us + 低 0.3us */
        .flags = { .msb_first = 1 }, /* 每字节高位先发 */
    }; /* 编码配置结束 */
    err = rmt_new_bytes_encoder(&enc_cfg, &s_encoder); /* 创建字节编码器 */
    if (err != ESP_OK) { /* 创建失败 */
        ESP_LOGE(TAG, "new bytes encoder failed: %s", esp_err_to_name(err)); /* 日志报错 */
        return err; /* 返回错误 */
    } /* 失败分支结束 */

    err = rmt_enable(s_chan); /* 使能 RMT 通道 */
    if (err != ESP_OK) { /* 使能失败 */
        ESP_LOGE(TAG, "enable failed: %s", esp_err_to_name(err)); /* 日志报错 */
        return err; /* 返回错误 */
    } /* 失败分支结束 */

    /* WS2812B 上电会保持上一次的颜色 (内部锁存)，必须在初始化完成后立刻
     * 发送一帧全黑，否则开机瞬间会闪现残留颜色。这里直接复用 ws2812_set
     * 发送 GRB(0,0,0)，内部 rmt_tx_wait_all_done 保证阻塞完成 */
    ws2812_set(0, 0, 0); /* 自举: 主动熄灭, 清除任何上电残留颜色 */
    ESP_LOGI(TAG, "init ok: gpio=%d, LED forced off", gpio_num); /* 日志: 初始化完成 */
    return ESP_OK; /* 返回成功 */
} /* 函数结束 */

void ws2812_set(uint8_t r, uint8_t g, uint8_t b) /* 输出一个灯珠颜色 (GRB) */
{ /* 函数体开始 */
    if (s_chan == NULL || s_encoder == NULL) { /* 未初始化 */
        return; /* 静默忽略 */
    } /* 未初始化分支结束 */
    uint8_t grb[3] = { g, r, b }; /* WS2812B 数据顺序: G R B */
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 }; /* 单次发送 (不循环) */
    if (rmt_transmit(s_chan, s_encoder, grb, sizeof(grb), &tx_cfg) != ESP_OK) { /* 提交发送失败 */
        return; /* 忽略 (grb 生命周期由阻塞等待保证) */
    } /* 发送失败分支结束 */
    rmt_tx_wait_all_done(s_chan, -1); /* 阻塞至发送完成, 保证 grb 缓冲在其间有效 */
} /* 函数结束 */
