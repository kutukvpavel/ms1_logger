/**
 * @file my_rf.cpp
 * @author MSU
 * @brief RF (BLE) communication interface abstraction C++ wrapper
 * @version 0.1
 * @date 2024-11-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "my_rf.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "macros.h"
#include "my_ble.h"

/// @brief ESP debug console log tag
static const char TAG[] = "MY_RF";

TaskHandle_t rf_task_handle;
QueueHandle_t rf_notification_queue;

/// @brief RF task notification message kinds
enum rf_task_arg_kind
{
    measured_value, ///< Measured hydrogen concenctration value changed
    measurement_state, ///< Device state changed
    trigger
};
/// @brief RF task notification queue message type
struct rf_task_arg_t
{
    rf_task_arg_kind kind; ///< See rf_task_arg_kind
    float value; ///< Used with measured_value notification kind
    my_rf::measurement_states state; ///< Used with measurement_state and trigger notification kinds
};

_BEGIN_STD_C
int gatt_svc_es_trigger_cb(uint8_t trigger)
{
    static rf_task_arg_t buffer = { .kind = rf_task_arg_kind::trigger };

    buffer.state = trigger > 0 ? my_rf::measurement_states::measuring : my_rf::measurement_states::idle;
    if (xQueueSend(rf_notification_queue, &buffer, 0) != pdTRUE) return BLE_ERR_CTLR_BUSY;
    return BLE_ERR_SUCCESS;
}
_END_STD_C

/// @brief RF task body function. Forwards data and state changed notifications to the BLE server.
/// @param arg Not used
static void rf_task(void* arg)
{
    static BaseType_t xResult;
    static rf_task_arg_t buffer;

    for (;;)
    {
        xResult = xQueueReceive(rf_notification_queue, &buffer, pdMS_TO_TICKS(1000));
        if (xResult == pdTRUE)
        {   
            switch (buffer.kind)
            {
            case rf_task_arg_kind::measured_value:
                my_ble_set_measured(buffer.value);
                break;
            case rf_task_arg_kind::measurement_state:
                my_ble_set_measurement_state(static_cast<uint8_t>(buffer.state));
                break;
            case rf_task_arg_kind::trigger:
                if (my_rf::trigger_callback) my_rf::trigger_callback(buffer.state);
                break;
            default:
                ESP_LOGW(TAG, "Urecognized RF task argument kind");
                break;
            }
        }
        else
        {
            vTaskDelay(1);
        }
        //Supervize state
        my_ble_supervize_state();
    }
}

/// @brief Enqueue a new notification into the RF notification queue
/// @param arg RF task argument structure
static void send_notif_queue(rf_task_arg_t* arg)
{
    if (xQueueSend(rf_notification_queue, arg, pdMS_TO_TICKS(1000)) != pdTRUE) // Queue copies arg into its buffer, so local scope is ok
        ESP_LOGW(TAG, "Notification queue is full");
}

namespace my_rf
{
    void (*trigger_callback)(measurement_states) = NULL;

    /// @brief Initialize RF abstraction layer, NimBLE stack, GAP and GATT servers, start required tasks.
    /// @param info Device info
    /// @return See my_ble_init and esp_bt_sleep_enable
    esp_err_t init(dev_info_t* info)
    {
        rf_notification_queue = xQueueCreate(8, sizeof(rf_task_arg_t));
        assert(rf_notification_queue);
        xTaskCreatePinnedToCore(rf_task, "rf_task", 4096, NULL, 1, &rf_task_handle, 0);
        assert(rf_task_handle);
        my_ble_manufacturer_name_str = info->manufacturer;
        my_ble_model_str = info->model;
        my_ble_serial_number_str = info->sn;
        my_ble_fw_revision_str = info->fw_rev;
        my_ble_pcb_revision_str = info->pcb_rev;
        return ESP_ERROR_CHECK_WITHOUT_ABORT(my_ble_init(info->bt_name));
    }

    /// @brief Remove all bonded devices from BLE NVS storage (not implemented).
    void clear_persistent()
    {
        my_ble_remove_all_bonded_devices();
    }
    /// @brief Call this when a new H2 concentration value is calculated.
    /// @param v H2 conc, ppm
    void notify_measurement_changed(float v)
    {
        rf_task_arg_t arg =
        {
            .kind = rf_task_arg_kind::measured_value,
            .value = v
        };
        send_notif_queue(&arg);
    }
    /// @brief Call this when main state machine state changes
    /// @param s Main state machine state
    void notify_state_changed(measurement_states s)
    {
        rf_task_arg_t arg =
        {
            .kind = rf_task_arg_kind::measurement_state,
            .state = s
        };
        send_notif_queue(&arg);
    }
    /// @brief Print all bonded devices' information into the debug console (not implemented)
    void show_bonded_devices()
    {
        my_ble_show_bonded_devices();
    }
}