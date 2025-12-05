idf_component_register(SRCS "wifi.c" "mqtt.c" "blink_example_main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES driver
                    REQUIRES vl53l0x ssd1306 nvs_flash esp_wifi esp_event esp_netif mqtt json driver esp_timer)