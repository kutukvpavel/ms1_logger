idf_component_register(
    SRCS "gatt_svr.c" 
        "my_ble.c"
        "my_rf.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt macros nvs_flash esp_pm)
