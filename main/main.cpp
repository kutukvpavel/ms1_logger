#include "sdcard.h"
#include "ms.h"

#include <_ansi.h>
#include <esp_log.h>

static const char TAG[] = "MAIN";

void result_saving_task(void* arg)
{
    static ms::export_data_t data;

    while (1)
    {
        if (xQueueReceive(*ms::results_queue, &data, portMAX_DELAY) != pdTRUE) continue;
        ESP_ERROR_CHECK_WITHOUT_ABORT(sd::append_result(&data));
    }
}

_BEGIN_STD_C
void app_main(void)
{
    //Initialize
    sd::init();
    ms::init();
    ESP_LOGD(TAG, "Init complete");

    //Setup MS and discard first measurement
    ESP_ERROR_CHECK_WITHOUT_ABORT(ms::set_sensing_range(ms::range_relay_state_t::RANGE_HIGH_Z));
    ms::wait();
    for (size_t i = 0; i < 2; i++)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(ms::set_heater_voltage(2));
        ms::wait();
    }
    xQueueReset(*ms::results_queue);
    ESP_LOGD(TAG, "MS init complete, starting cycles");
    
    //Perform mesurement cycles
    while (1)
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(ms::perform_transaction());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
_END_STD_C
