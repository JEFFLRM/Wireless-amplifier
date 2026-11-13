/**
 * tcp_log.h - TCP 远程日志输出 (ESP-IDF v5.x)
 *
 * 在指定端口 (默认 3333) 开一个 TCP server, PC 端 telnet/nc 连上后即可实时接收 ESP_LOG 输出.
 * 不影响原有 USB 串口日志 (vprintf 回调同时写两处).
 *
 *   PC 端: nc <esp32_ip> 3333    或    telnet <esp32_ip> 3333
 *   单次仅支持 1 个客户端; 新连接会踢掉旧的, 旧连接自动重连即可.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief 启动 TCP 远程日志. 创建 FreeRTOS 任务监听端口, 注册 esp_log vprintf 回调.
 *        应在 esp_netif_init() 之后调用 (不依赖 WiFi 是否已连, bind 0.0.0.0).
 * @param port 监听端口, 传 0 用默认 3333
 */
esp_err_t tcp_log_init(uint16_t port);
