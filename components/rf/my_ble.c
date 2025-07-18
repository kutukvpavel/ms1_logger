/**
 * @file my_ble.c
 * @author MSU
 * @brief BLE GAP (advertising) and GATT server control code, adapted from ESP-IDF examples.
 * @date 2024-11-28
 * 
 */

#include "my_ble.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

struct my_connection
{
    uint16_t id;
    bool active;
} typedef my_connection_t;

static const char *tag = "NimBLE_Interface";
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
static bool is_advertising = false;
static struct ble_gap_adv_params adv_params;
static TaskHandle_t nimble_task_handle = NULL;

my_connection_t current_connection = { 0, 0 };
SemaphoreHandle_t gap_mutex;
SemaphoreHandle_t gatts_mutex;

void ble_store_config_init(void);

void
print_bytes(const uint8_t *bytes, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}


/// @brief Logs information about a connection to the console.
/// @param desc 
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

/// @brief Enables advertising with the following parameters:
///    o General discoverable mode.
///    o Undirected connectable mode.
static void
bleprph_advertise(void)
{
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(0x181A)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.appearance = 0x0540; //Generic sensor
    fields.appearance_is_present = true;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 0; //x0.625ms = ~100mS
    adv_params.itvl_max = 0; // = ~300mS
    my_ble_start_advertising();
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
            current_connection.active = true;
            current_connection.id = event->connect.conn_handle;
        }
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        current_connection.active = false;
        current_connection.id = 0;
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        current_connection.id = event->conn_update.conn_handle;
        return 0;
    
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
        *event->conn_update_req.self_params = *event->conn_update_req.peer_params;
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
        //bleprph_advertise();
        is_advertising = false;
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(tag, "PASSKEY_ACTION_EVENT started \n");
        struct ble_sm_io pkey = {0};

        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            pkey.action = event->passkey.params.action;
            pkey.passkey = 123456; // This is the passkey to be entered on peer
            ESP_LOGI(tag, "Enter passkey %" PRIu32 " on the peer side", pkey.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %i\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            ESP_LOGI(tag, "Passkey on device's display: %" PRIu32, event->passkey.params.numcmp);
            ESP_LOGI(tag, "Accept or reject the passkey through console in this format -> key Y or key N");
            pkey.action = event->passkey.params.action;
            pkey.numcmp_accept = 0;
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %i\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
            static uint8_t tem_oob[16] = {0};
            pkey.action = event->passkey.params.action;
            for (int i = 0; i < 16; i++) {
                pkey.oob[i] = tem_oob[i];
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %i\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            ESP_LOGI(tag, "Enter the passkey through console in this format-> key 123456");
            pkey.action = event->passkey.params.action;
            pkey.passkey = 123456;
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %i\n", rc);
        }
        return 0;

    default:
        ESP_LOGI(tag, "Undeclared BLE event: %u", event->type);
    }

    return 0;
}

/// @brief Executes GAP event handler in a thread-safe manner 
/// @param event Event
/// @param arg Argument
/// @return 
static int bleprph_gap_event_wrapper(struct ble_gap_event* event, void *arg)
{
    BaseType_t taken = xSemaphoreTakeRecursive(gap_mutex, pdMS_TO_TICKS(500));
    int ret = 0;
    if (taken == pdTRUE)
    {
        ret = bleprph_gap_event(event, arg);
        xSemaphoreGiveRecursive(gap_mutex);
    }
    return ret;
}

/// @brief BLE reset event handler (simply logs it to console)
/// @param reason See NimBLE documentation
static void
bleprph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}
/// @brief BLE sync event handler
static void
bleprph_on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
    bleprph_advertise();
}

/// @brief NimBLE host task body function
/// @param param Not used
void bleprph_host_task(void *param)
{
    nimble_task_handle = xTaskGetCurrentTaskHandle();
    assert(nimble_task_handle);
    ESP_LOGI(tag, "BLE Host Task Started");
    while (1)
    {
        /* This function will return only when nimble_port_stop() is executed */
        nimble_port_run();
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdTRUE);
    }
}

/// @brief Not implemented
void my_ble_remove_all_bonded_devices(void)
{
    
}
/// @brief Not implemented
void my_ble_show_bonded_devices(void)
{

}
/// @brief Stop BLE GAP advertising (if advertising)
void my_ble_stop_advertising(void)
{
    if (is_advertising)
    {
        int rc = ble_gap_adv_stop();
        ESP_LOGI(tag, "Adv disable result: %i", rc);
        if (rc == 0 || rc == BLE_HS_EALREADY) is_advertising = false;
    } 
}
/// @brief Start BLE GAP advertising (only if not already advertising, if no central is connected and if sleep is not requested)
void my_ble_start_advertising(void)
{
    if (is_advertising || current_connection.active) 
    {
        /*ESP_LOGI(tag, "Skip Adv start: is_adv = %i, conn_active = %i",
            is_advertising, current_connection.active);*/
        return;
    }
    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                                &adv_params, bleprph_gap_event_wrapper, NULL);
    ESP_LOGI(tag, "Adv enable result: %i", rc);
    if (rc == 0 || rc == BLE_HS_EALREADY) is_advertising = true;
}
/// @brief Terminate current connection (if any)
void my_ble_disconnect(void)
{
    if (current_connection.active)
        ble_gap_terminate(current_connection.id, 0x16); //HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST
}
/// @brief Set measured H2 concentration value into the GATT server
/// @param v H2 conc, ppm
void my_ble_set_measured(float v)
{
    *(float*)(gatt_svr_measured_chr_notif.val) = v;
    if (current_connection.active) ble_gatts_notify(current_connection.id, gatt_svr_measured_chr_notif.handle);
}
/// @brief Set measurement state into the GATT server
/// @param s State
void my_ble_set_measurement_state(uint8_t s)
{
    gatt_svr_measured_trigger_desc = s < 0xFF ? s : 0;
    *(uint8_t*)(gatt_svr_measurement_state_chr_notif.val) = s;
    if (current_connection.active) ble_gatts_notify(current_connection.id, gatt_svr_measurement_state_chr_notif.handle);
}

/// @brief Supervize BLE advertising state (enable it if should advertise, but an event was missed, causing the device to be stuck silent)
void my_ble_supervize_state(void)
{
    is_advertising = ble_gap_adv_active();
    bool should_advertize = !current_connection.active;
    if (should_advertize) my_ble_start_advertising();
}

/// @brief Halt NimBLE host task
void my_ble_stop(void)
{
    nimble_port_stop();
}
/// @brief Resume NimBLE host task
void my_ble_resume(void)
{
    assert(nimble_task_handle);
    xTaskNotifyGive(nimble_task_handle);
}

/// @brief Initialize NimBLE stack, GATT server, BLE NVS storage; set device name and start NimBLE host task
/// @param name Bluetooth device name
/// @return ESP_OK, logs error codes to debug console, otherwise panics (if nimble_port_init fails)
esp_err_t my_ble_init(const char* name)
{
    int rc;
    gatts_mutex = xSemaphoreCreateRecursiveMutex();
    assert(gatts_mutex);
    gap_mutex = xSemaphoreCreateRecursiveMutex();
    assert(gap_mutex);

    ESP_ERROR_CHECK(nimble_port_init());
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_sc = 0;

    rc = gatt_svr_init();
    if (rc != 0)
    {
        ESP_LOGE(tag, "GATT SVR init error: %i", rc);
    }

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set(name);
    if (rc != 0)
    {
        ESP_LOGE(tag, "GATT SVR init error: %i", rc);
    }

    /* XXX Need to have template for store */
    ble_store_config_init();
    nimble_port_freertos_init(bleprph_host_task);
    return ESP_OK;
}
