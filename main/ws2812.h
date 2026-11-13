/**
 * ws2812.h - WS2812B 单总线 RGB LED 驱动 (ESP32-C5, RMT TX)
 *
 * 用 RMT 的 bytes encoder 编码 WS2812B 时序 (GRB 顺序, MSB first),
 * 分辨率 10MHz (1 tick = 0.1us): T0H=0.3us T0L=0.9us / T1H=0.9us T1L=0.3us。
 * 单灯珠: 发送 3 字节 GRB 后拉低 >50us 即锁存 (帧间隔天然满足, 无需额外 reset)。
 */
#ifndef WS2812_H /* 防重复包含: 首次定义 */
#define WS2812_H /* 防重复包含: 定义完成 */

#include <stdint.h> /* uint8_t 等定长类型 */
#include "esp_err.h" /* esp_err_t 错误码类型 */

#ifdef __cplusplus /* C++ 兼容: 进入 C 链接域 */
extern "C" {
#endif

/** 初始化 WS2812B 驱动 (创建 RMT TX 通道与编码器)。重复调用安全。 */
esp_err_t ws2812_init(int gpio_num); /* gpio_num: WS2812B 数据脚 */

/** 设置灯珠颜色 (0~255)。未初始化时静默忽略。阻塞至发送完成。 */
void ws2812_set(uint8_t r, uint8_t g, uint8_t b); /* r/g/b 各 0~255 */

#ifdef __cplusplus /* C++ 兼容: 关闭 C 链接域 */
}
#endif

#endif /* WS2812_H */
