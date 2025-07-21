#include "my_sdcard.h"
#include "ms.h"
#include "my_rf.h"
#include "main.h"

#include <_ansi.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <nvs_flash.h>

#define LED_PIN GPIO_NUM_33

static const char TAG[] = "MAIN";
static const float heater_target_voltage = 1.0f;
static const size_t heater_heatup_steps = 10;
static my_rf::dev_info_t device_info = {
    .bt_name = "MSU_SENSE",
    .manufacturer = "MSU",
    .model = "Archipelag",
    .sn = "N/A",
    .fw_rev = "v0.1",
    .pcb_rev = "N/A"
};
static bool measure = true;
static TaskHandle_t result_saving_task_handle = NULL;

static void result_saving_task(void* arg)
{
    static ms::export_data_t data;

    while (1)
    {
        if (xQueueReceive(*ms::results_queue, &data, portMAX_DELAY) != pdTRUE) continue;
        printf(RESULTS_FORMAT, data.heater_temperature, data.concentration);
        ESP_ERROR_CHECK_WITHOUT_ABORT(sd::append_result(&data));
        my_rf::notify_measurement_changed(data.concentration);
    }
}

void rf_trigger_callback(my_rf::measurement_states state)
{
    bool new_state = (state == my_rf::measurement_states::measuring);
    if (new_state != measure)
    {
        measure = new_state;
        my_rf::notify_state_changed(measure ? my_rf::measurement_states::measuring : my_rf::measurement_states::idle);
    }
}

_BEGIN_STD_C
void app_main(void)
{
    static const float heater_heatup_step_voltage = heater_target_voltage / (float)heater_heatup_steps;
    static bool led_state = false;

    //Initialize
    ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT_OD));
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));
    ESP_ERROR_CHECK(ms::init());
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
    vTaskDelay(pdMS_TO_TICKS(3000)); //Wait for LDO reset circuit to enable RS422 drivers
    ESP_ERROR_CHECK(nvs_flash_init());
    my_rf::trigger_callback = rf_trigger_callback;
    ESP_ERROR_CHECK(my_rf::init(&device_info));
    ESP_ERROR_CHECK(sd::init());
    ESP_LOGI(TAG, "Init complete");
    vTaskDelay(pdMS_TO_TICKS(3000));

    //Setup MS and discard first measurement
    ESP_ERROR_CHECK_WITHOUT_ABORT(ms::set_sensing_range(ms::range_relay_state_t::RANGE_LOW_Z));
    ms::wait();
    vTaskDelay(pdMS_TO_TICKS(200));
    for (size_t i = 0; i < heater_heatup_steps; i++)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(ms::set_heater_voltage(heater_heatup_step_voltage * i));
        ms::wait();
        vTaskDelay(pdMS_TO_TICKS(200));
        xQueueReset(*ms::results_queue);
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(ms::set_heater_voltage(heater_target_voltage));
    ms::wait();
    xQueueReset(*ms::results_queue);
    xTaskCreate(result_saving_task, "SAVE", 3072, NULL, 1, &result_saving_task_handle);
    assert(result_saving_task_handle);
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "MS init complete, starting cycles");
    my_rf::notify_state_changed(my_rf::measurement_states::measuring);
    
    //Perform mesurement cycles
    while (1)
    {
        static bool last_measure = true;

        bool m = measure; //Thread-safety
        if (m != last_measure)
        {
            if (m)
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(sd::create_new_file());
            }
            else
            {
                ESP_ERROR_CHECK_WITHOUT_ABORT(sd::finalize_file());
            }
            last_measure = m;
        }
        if (m)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(ms::perform_transaction());
            led_state = !led_state;
            ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(LED_PIN, led_state ? 1 : 0));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
_END_STD_C
