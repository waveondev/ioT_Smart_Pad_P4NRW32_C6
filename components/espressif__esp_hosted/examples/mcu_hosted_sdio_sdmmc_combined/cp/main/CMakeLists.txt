idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES espressif__esp_hosted nvs_flash esp_wifi
)