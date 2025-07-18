/**
 * @file my_ble.h
 * @author MSU
 * @brief Bluetooth Low Energy broadcaster
 * @version 0.1
 * @date 2022-10-25
 * 
 */

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include "nimble/ble.h"
#include "modlog/modlog.h"

#ifdef __cplusplus
    extern "C" {
#endif

/**
 * @brief NimBLE struct declaration to keep the compiler happy (something is wrong with ESP-IDF BLE stack includes)
 * 
 */
///@{
    struct ble_hs_cfg;
    struct ble_gatt_register_ctxt;
///@}

    /// @brief BLE characteristic value changed notification structure
    struct 
    {
        void* val; ///< External pointer to a new value
        uint16_t handle; ///< Internal GATT server characteristic handle
    } typedef chr_notif_t;

    void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
    int gatt_svr_init(void);
    int gatt_svc_es_trigger_cb(uint8_t trigger);

    extern const char* my_ble_manufacturer_name_str;
    extern const char* my_ble_model_str;
    extern const char* my_ble_serial_number_str;
    extern const char* my_ble_fw_revision_str;
    extern const char* my_ble_pcb_revision_str;
    extern chr_notif_t gatt_svr_measured_chr_notif;
    extern chr_notif_t gatt_svr_measurement_state_chr_notif;
    extern uint8_t gatt_svr_measured_trigger_desc;
    extern SemaphoreHandle_t gap_mutex;
    extern SemaphoreHandle_t gatts_mutex;

    esp_err_t my_ble_init(const char* name);

    void my_ble_supervize_state(void);
    void my_ble_remove_all_bonded_devices();
    void my_ble_show_bonded_devices(void);
    void my_ble_stop_advertising(void);
    void my_ble_start_advertising(void);
    void my_ble_disconnect(void);
    void my_ble_set_measured(float v);
    void my_ble_set_measurement_state(uint8_t s);

    void my_ble_stop(void);
    void my_ble_resume(void);

#ifdef __cplusplus
    }
#endif