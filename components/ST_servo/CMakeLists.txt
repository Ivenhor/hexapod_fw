idf_component_register(
    SRCS "src/st_servo.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_uart esp_timer
)
