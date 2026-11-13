idf_component_register(SRCS "tcp_log.c" "ota_update.c" "ble_provision.c" "receiver.c" "acm8815.c" "ws2812.c"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash lwip driver esp_driver_i2s esp_driver_gpio esp_driver_i2c esp_driver_rmt bt cjson esp_http_server app_update
                       INCLUDE_DIRS "")

