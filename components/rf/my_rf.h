/**
 * @file my_rf.h
 * @author MSU
 * @brief RF (BLE) communication interface abstraction
 * @version 0.1
 * @date 2022-10-25
 * 
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#ifdef __cplusplus
/// @brief BLE communication interface public API
namespace my_rf
{
    /// @brief Device states to be reported by BLE communication interface
    enum measurement_states : uint8_t
    {
        idle = 0, ///< Device is ready
        measuring, ///< The device is perofrming measurements
        initializing = 0xFF ///< The device has not yet finished firmware initialization
    };

    /// @brief Pointers to device information strings for BLE communication interface
    struct dev_info_t
    {
        const char* bt_name; ///< Bluetooth device name
        const char* manufacturer; ///< Device manufacturer (see my_dev_info_t)
        const char* model; ///< Device model, should be used to differentiate between dekstop and portable variants (see my_dev_info_t)
        const char* sn; ///< Device serial number (see my_dev_info_t)
        const char* fw_rev; ///< Device firmware revision string (see HAL)
        const char* pcb_rev; ///< Device PCB hardware revision (see my_dev_info_t)
    };

    extern void (*trigger_callback)(measurement_states);

    esp_err_t init(dev_info_t* name);

    void clear_persistent();
    void notify_measurement_changed(float v);
    void notify_state_changed(measurement_states s);
    void show_bonded_devices();
}
#endif
