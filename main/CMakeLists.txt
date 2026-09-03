idf_component_register(SRCS "main.c" "camera.c" "stream_uart.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_camera driver freertos)
