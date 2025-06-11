idf_component_register(SRCS "main.c" "display.c" "wifi.c"
                    PRIV_REQUIRES esp_event nvs_flash esp_wifi esp_http_server driver esp_netif
                    INCLUDE_DIRS "."
                    EMBED_FILES index.html
                    )
