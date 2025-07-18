idf_component_register(SRCS main.cpp
                        sdcard.cpp
                        ms.cpp
                       INCLUDE_DIRS "."
                       REQUIRES littlefs fatfs esp_driver_sdspi esp_driver_uart
                       )
