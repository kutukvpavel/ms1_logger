#pragma once

#include <esp_err.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifdef __cplusplus
namespace ms
{
    struct export_data_t
    {
        float heater_temperature;
        float concentration;
    };
    enum range_relay_state_t : uint8_t {
        RANGE_HIGH_Z = 0b00,
        RANGE_MID_Z = 0b10,
        RANGE_LOW_Z = 0b11
    };

    extern const SemaphoreHandle_t* const transaction_complete;
    extern const QueueHandle_t* const results_queue;

    esp_err_t init();
    esp_err_t set_heater_voltage(float volts);
    esp_err_t set_sensing_range(range_relay_state_t range);
    esp_err_t perform_transaction();
    esp_err_t wait(TickType_t timeout = portMAX_DELAY);
} // namespace ms
#endif