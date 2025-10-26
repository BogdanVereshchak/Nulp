idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        bmp280 
        ens160 
        hall_sensor
        esp_http_client
        esp_https_ota
        nvs_flash
        esp_wifi
        esp_timer
        json
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
