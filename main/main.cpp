#include "sdcard.h"
#include "ms.h"
#include "my_rf.h"

#include <_ansi.h>
#include <esp_log.h>

static const char TAG[] = "MAIN";
static my_rf::dev_info_t device_info = {
    .bt_name = "MSU_SENSE",
    .manufacturer = "MSU",
    .model = "Archipelag",
    .sn = "N/A",
    .fw_rev = "v0.1",
    .pcb_rev = "N/A"
};
static bool measure = true;

static void result_saving_task(void* arg)
{
    static ms::export_data_t data;

    while (1)
    {
        if (xQueueReceive(*ms::results_queue, &data, portMAX_DELAY) != pdTRUE) continue;
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
    //Initialize
    my_rf::trigger_callback = rf_trigger_callback;
    my_rf::init(&device_info);
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
    my_rf::notify_state_changed(my_rf::measurement_states::measuring);
    
    //Perform mesurement cycles
    while (1)
    {
        if (measure) ESP_ERROR_CHECK_WITHOUT_ABORT(ms::perform_transaction());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
_END_STD_C
