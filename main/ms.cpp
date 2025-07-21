#include "ms.h"

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include <math.h>

#define TXD_PIN GPIO_NUM_0
#define RXD_PIN GPIO_NUM_16
#define RX_BUF_SIZE 256
#define UART_NUM UART_NUM_1

#define MS_BAUDRATE 115200

#define MAKE_PACKET(t) packet_t<t>* packet = reinterpret_cast<packet_t<t>*>(inbound_buffer)
#define TAKE_LOCK(t) xSemaphoreTake(transaction_complete_semaphore, t)

namespace ms
{
    static const char TAG[] = "MS";

    // CONSTANTS
    static const float CONVERT_TO_FLOAT_COEFF = 16777215.0;
    static const float CONVERT_TO_FLOAT_R_COEFF = 0.01;
    static const float RS1 = 2.9993;
    static const float RS2 = 2.00438;
    static const float K = 4.124;
    static const float R4 = 100000.0;
    static const float B  = -0.84789444;
    static const float C = 5.00644098;
    static const float Rn0  = 15.7;
    static const float Rc = 0.1;
    static const float t0 = 40.0;
    static const float alpha = 0.003;
    static const float reciprocal_step = 65535.0 / 5.0;
    static const float Uinf = 2.5 + K * (2.5 - RS2);

    /**
     * @brief Protocol
     * 
     */
    static const size_t sensors_number = 4;
    template <typename T> struct __packed packet_t
    {
        const uint8_t flag_start[3] = { 0xAA, 0x55, 0xAA };
        T payload;
        const uint8_t flag_end[2] = { 0x0D, 0x0A };
    };
    //Outbound packets
    enum cmd_designator_t : uint8_t {
        SET_HEATER_U = 0x08,
        SET_HEATER_R = 0x88,
        SET_RANGE_RELAY = 0x20
    };
    struct __packed cmd_set_heater_t {
        uint16_t value[sensors_number];
    };
    struct __packed cmd_set_heater_u_t
    {
        const cmd_designator_t designator = cmd_designator_t::SET_HEATER_U;
        cmd_set_heater_t voltages;
    };
    struct __packed cmd_set_heater_r_t
    {
        const cmd_designator_t designator = cmd_designator_t::SET_HEATER_R;
        cmd_set_heater_t resistances;
    };
    struct __packed cmd_set_range_relay_t
    {
        const cmd_designator_t designator = cmd_designator_t::SET_RANGE_RELAY;
        uint8_t state; //4 flags according to range_relay_state_t
    };
    //Inbound packets
    struct __packed sensor_data_t
    {
        uint16_t heater_resistance;
        uint32_t sensor_voltage; //Take only 24 bits
    };
    struct __packed resp_common_t
    {
        sensor_data_t response[sensors_number];
    };  
    struct __packed resp_range_relay_t
    {
        uint8_t response;
    };

    //Other
    uint8_t inbound_buffer[sizeof(packet_t<resp_common_t>) * 2 + 1] = {};
    TaskHandle_t receiver_task_handle = NULL;
    QueueHandle_t receiver_response_queue = NULL;
    SemaphoreHandle_t transaction_complete_semaphore = NULL;
    const SemaphoreHandle_t* const transaction_complete = &transaction_complete_semaphore;
    QueueHandle_t results_queue_queue = NULL;
    const QueueHandle_t* const results_queue = &results_queue_queue;
    float last_heater_voltage = 0;

    /**
     * PRIVATE METHODS
     * 
     */

    static float convert_to_resistance(uint32_t u)
    {
        float u_float = (float)u / CONVERT_TO_FLOAT_COEFF * 5.0;
        if (u_float >= Uinf)
        {
            return 7008604081291.181;
        }
        else
        {
            return (((RS1 - RS2) * R4 / ((2.5 + 2.5 * K - u_float) / K - RS2) - R4));
        }
    }
    static float convert_to_heater_resistance(uint16_t r)
    {
        return r * CONVERT_TO_FLOAT_R_COEFF;
    }
    static float convert_to_concentration(float r)
    {
        return pow(10, B * log10(r) + C);
    }
    static float convert_to_temperature(float rn)
    {
        return ((rn - Rc) / (Rn0 - Rc) - 1) / alpha + t0;
    }
    template <typename T> static int send(const packet_t<T>* packet_ptr)
    {
        if (TAKE_LOCK(pdMS_TO_TICKS(2000)) != pdTRUE)
        {
            ESP_LOGE(TAG, "Can't take transaction semaphore!");
            return 0;
        }
        ESP_LOGD(TAG, "Transaction semaphore taken");
        auto payload = reinterpret_cast<const uint8_t*>(&(packet_ptr->payload));
        uart_flush(UART_NUM);
        if (xQueueSend(receiver_response_queue, &(payload[0]), 0) != pdTRUE)
        {
            ESP_LOGE(TAG, "Receiver queue is full!");
            return 0; //First byte of the payload is the command designator
        }
        ESP_LOGD(TAG, "Receiver queue is not full");
        return uart_write_bytes(UART_NUM, packet_ptr, sizeof(*packet_ptr));
    }
    static void process_answer(cmd_designator_t request_designator)
    {
        switch (request_designator)
        {
        case cmd_designator_t::SET_HEATER_R:
        case cmd_designator_t::SET_HEATER_U:
        {
            MAKE_PACKET(resp_common_t);
            static sensor_data_t first_sensor_data = packet->payload.response[0];
            static export_data_t export_data;
            export_data.heater_temperature = convert_to_temperature(convert_to_heater_resistance(first_sensor_data.heater_resistance));
            export_data.concentration = convert_to_concentration(convert_to_resistance(first_sensor_data.sensor_voltage & 0xFFFFFFu));
            if (xQueueSend(results_queue_queue, &export_data, pdMS_TO_TICKS(1000)) != pdTRUE) ESP_LOGW(TAG, "Results queue is full!");
            break;
        }
        case cmd_designator_t::SET_RANGE_RELAY:
        {
            MAKE_PACKET(resp_range_relay_t);
            if (packet->payload.response == 0x20) ESP_LOGD(TAG, "Range relays are set.");
            else ESP_LOGE(TAG, "Unexpected response for SET_RANGE_RELAY!");
            break;
        }
        default:
            ESP_LOGW(TAG, "Unknown answer type!");
            return;
        }        
    }
    static size_t get_response_length(cmd_designator_t request_designator)
    {
        switch (request_designator)
        {
        case cmd_designator_t::SET_HEATER_R:
        case cmd_designator_t::SET_HEATER_U:
            return sizeof(packet_t<resp_common_t>);
        case cmd_designator_t::SET_RANGE_RELAY:
            return sizeof(packet_t<resp_range_relay_t>);
        default:
            break;
        }
        return 0;
    }
    static void rx_task(void *arg)
    {
        static cmd_designator_t last_command_designator;
        while (1) {
            if (xQueueReceive(receiver_response_queue, &last_command_designator, portMAX_DELAY) != pdPASS) continue;
            static const size_t resp_len = get_response_length(last_command_designator);
            if (resp_len > 0)
            {
                static const int rxBytes = uart_read_bytes(UART_NUM, inbound_buffer, resp_len, pdMS_TO_TICKS(1000));
                if (rxBytes < resp_len)
                {
                    ESP_LOGE(TAG, "Incomplete answer (%i bytes)!", rxBytes);
                }
                else
                {
                    inbound_buffer[rxBytes] = 0;
                    ESP_LOGD(TAG, "Read %d bytes", rxBytes);
                    process_answer(last_command_designator);
                }
            }
            else
            {
                ESP_LOGE(TAG, "Unknown response type!");
                uart_read_bytes(UART_NUM, inbound_buffer, sizeof(inbound_buffer), pdMS_TO_TICKS(2000));
            }
            uart_flush(UART_NUM);
            xSemaphoreGive(transaction_complete_semaphore);
        }
    }
    static esp_err_t control_sending(int sent, int sz)
    {
        if (sent != sz)
        {
            ESP_LOGE(TAG, "Partial sending: %i instead of %i!", sent, sz);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    /**
     * PUBLIC METHODS
     */

    esp_err_t init()
    {
        static_assert(sizeof(inbound_buffer) > sizeof(packet_t<resp_common_t>) &&
            sizeof(inbound_buffer) > sizeof(packet_t<resp_range_relay_t>));

        //Configure UART
        const uart_config_t uart_config = {
            .baud_rate = MS_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT
        };
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0)); // We won't use a buffer for sending data.
        ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        //Create tasks
        receiver_response_queue = xQueueCreate(2, sizeof(cmd_designator_t));
        assert(receiver_response_queue);
        transaction_complete_semaphore = xSemaphoreCreateBinary();
        assert(transaction_complete_semaphore);
        xSemaphoreGive(transaction_complete_semaphore);
        results_queue_queue = xQueueCreate(8, sizeof(export_data_t));
        xTaskCreate(rx_task, "MS_RX", 3072, NULL, 1, &receiver_task_handle);
        assert(receiver_task_handle);
        return ESP_OK;
    }
    esp_err_t set_heater_voltage(float volts)
    {
        static packet_t<cmd_set_heater_u_t> packet = { .payload = { .voltages = { .value = { 0 } } } };

        if (volts < 0) volts = 0;
        uint32_t raw = (uint32_t)(volts * reciprocal_step);
        if (raw > UINT16_MAX) raw = UINT16_MAX;
        packet.payload.voltages.value[0] = raw;
        last_heater_voltage = volts;
        return control_sending(send(&packet), sizeof(packet));
    }
    esp_err_t set_sensing_range(range_relay_state_t range)
    {
        static packet_t<cmd_set_range_relay_t> packet = { .payload = { .state = { } } };

        packet.payload.state = 0;
        for (size_t i = 0; i < sensors_number; i++)
        {
            packet.payload.state |= (range << (i * 2));
        }
        return control_sending(send(&packet), sizeof(packet));
    }
    esp_err_t perform_transaction()
    {
        return set_heater_voltage(last_heater_voltage);
    }
    esp_err_t wait(TickType_t timeout)
    {
        BaseType_t taken = TAKE_LOCK(timeout);
        if (taken == pdTRUE) 
        {
            xSemaphoreGive(transaction_complete_semaphore);
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }
} // namespace ms