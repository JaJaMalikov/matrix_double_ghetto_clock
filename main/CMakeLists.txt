idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES esp_event nvs_flash bt driver
                    INCLUDE_DIRS ".")
