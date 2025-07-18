/**
 * @file gatt_svr.c
 * @author MSU
 * @brief BLE GATT server code, adapted from ESP-IDF examples. Environmental sensing ("es") service profile is used to report
 * measured hydrogen concentration. Device information ("info") service profile is used to report device information like
 * manufacturer, model, s/n and so on.
 * @date 2024-11-28
 * 
 */

#include "my_ble.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "macros.h"

/// @brief Simulate a managed array type in C
struct
{
    uint8_t* items;
    uint16_t len;
} typedef array_t;
/** Macro used to pass a single byte as a byte-array (see array_t) with length 1, required by some BLE APIs */
#define PASS_AS_BYTE_ARRAY(a) (&((array_t) { &(a), sizeof(a) }))
/** Macro used to wrap a static C-array into an array_t instance */
#define WRAP_BYTE_ARRAY(a) (&((array_t) { (a), sizeof(a) }))
/** Maximum string characteristic length expected by this API, must be greater than INFO_STR_MAX_LEN and BT_NAME_MAX_LEN in my_params */
#define CONST_STRING_MAX_LEN 32

/// @brief Debug log tag
static const char TAG[] = "GATT_SVR";

/**
 * @brief All UUIDs used by GATT server
 * 
 */
///@{
static const ble_uuid16_t gatt_svr_svc_sensing_uuid = BLE_UUID16_INIT(0x181A);
static const ble_uuid16_t gatt_svr_chr_measured_conc_uuid = BLE_UUID16_INIT(0x2BD3);
static const ble_uuid16_t gatt_svr_desc_chr_presentation_uuid = BLE_UUID16_INIT(0x2904);
static const ble_uuid16_t gatt_svr_desc_es_measurement_uuid = BLE_UUID16_INIT(0x290C);
static const ble_uuid16_t gatt_svr_desc_es_trigger_uuid = BLE_UUID16_INIT(0x290D);
static const ble_uuid16_t gatt_svr_chr_measurement_state_uuid = BLE_UUID16_INIT(0x2BBB);
static const ble_uuid16_t gatt_svr_desc_chr_user_desc_uuid = BLE_UUID16_INIT(0x2901);
static const ble_uuid16_t gatt_svr_svc_info_uuid = BLE_UUID16_INIT(0x180A);
static const ble_uuid16_t gatt_svr_chr_manufacturer_uuid = BLE_UUID16_INIT(0x2A29);
static const ble_uuid16_t gatt_svr_chr_model_uuid = BLE_UUID16_INIT(0x2A24);
static const ble_uuid16_t gatt_svr_chr_sn_uuid = BLE_UUID16_INIT(0x2A25);
static const ble_uuid16_t gatt_svr_chr_fw_uuid = BLE_UUID16_INIT(0x2A26);
static const ble_uuid16_t gatt_svr_chr_pcb_uuid = BLE_UUID16_INIT(0x2A27);
///@}

/**
 * @brief Pointers to device information strings passed from configuration API to this (BLE) API
 * 
 */
///@{
const char* my_ble_manufacturer_name_str;
const char* my_ble_model_str;
const char* my_ble_serial_number_str;
const char* my_ble_fw_revision_str;
const char* my_ble_pcb_revision_str;
///@}

/**
 * @brief Environmental sensing ("es") service characteristics and attributes.
 * Attributes are essentially constant, remaining statically-initialized for the whole duration of program execution.
 * Values can change and are initialized to some obviously "bad" values.
 */
///@{
static uint8_t gatt_svr_measured_attrs_desc[11] = { 
    0x00, 0x00, //Flags, reserved
    0x00, //Sampling = unspecified
    30, 0x00, 0x00, //Sampling interval = 30sec
    0x00, 0x00, 0x00, //Update interval = unapplicable
    0x00, //Application = unspecified
    40 //Uncertainty = 40x0.5% = 20%
 };
uint8_t gatt_svr_measured_trigger_desc = 0x00; //Trigger Inactive
#define SENS_UNIT_UUID 0x27C4 //ppm
static uint8_t gatt_svr_measured_pres_desc[] = {
    0x14, //IEEE-754
    0x00, //Exponent
    _HB(SENS_UNIT_UUID), _LB(SENS_UNIT_UUID), //Unit UUID
    0x01, //Description namespace (Bluetooth SIG)
    0x00, 0x00 //Description index (??)
 };
static uint8_t gatt_svr_measurement_state_desc[] = "Measurement state";
///@}

/// @brief GATT value buffer for last hydrogen concentration value reported to the BLE communication API by the main state machine
static float gatt_svr_measured_val = 0x0BADF00D;
/// @brief GATT value buffer for last device state reported to the BLE communication API by the main state machine
static uint8_t gatt_svr_measurement_state = 0xFF;
/// @brief Characteristic value changed notification instance for hydrogen concentration. Characterisitc handle default to NULL and will be initialized by NimBLE stack.
chr_notif_t gatt_svr_measured_chr_notif = { &gatt_svr_measured_val, 0 };
/// @brief Characteristic value changed notification instance for device state. Characterisitc handle default to NULL and will be initialized by NimBLE stack.
chr_notif_t gatt_svr_measurement_state_chr_notif = { &gatt_svr_measurement_state, 0 };


/* Some forward-declarations */
static int
gatt_svr_chr_access_wrapper(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);
static int
gatt_svr_desc_access_wrapper(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int
gatt_svr_chr_constant_wrapper(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);


/// @brief GATT server services definition, all the services, characteristics and attribute descriptors must be
/// added here to be taken into account by the NimBLE stack.
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service: ES */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_sensing_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /*** Characteristic: sensor measured value. */
                .uuid = &gatt_svr_chr_measured_conc_uuid.u,
                .access_cb = gatt_svr_chr_access_wrapper,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_measured_chr_notif.handle,
                .descriptors = (struct ble_gatt_dsc_def[])
                {
                    {
                        .uuid = &gatt_svr_desc_es_measurement_uuid.u,
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .access_cb = gatt_svr_desc_access_wrapper,
                        .arg = WRAP_BYTE_ARRAY(gatt_svr_measured_attrs_desc)
                    },
                    {
                        .uuid = &gatt_svr_desc_es_trigger_uuid.u,
                        .att_flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                        .access_cb = gatt_svr_desc_access_wrapper,
                        .arg = PASS_AS_BYTE_ARRAY(gatt_svr_measured_trigger_desc)
                    },
                    {
                        .uuid = &gatt_svr_desc_chr_presentation_uuid.u,
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .access_cb = gatt_svr_desc_access_wrapper,
                        .arg = WRAP_BYTE_ARRAY(gatt_svr_measured_pres_desc)
                    },
                    { 0 }
                }
            }, {
                /*** Characteristic: mesurement state */
                .uuid = &gatt_svr_chr_measurement_state_uuid.u,
                .access_cb = gatt_svr_chr_access_wrapper,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &gatt_svr_measurement_state_chr_notif.handle,
                .descriptors = (struct ble_gatt_dsc_def[])
                {
                    {
                        .uuid = &gatt_svr_desc_chr_user_desc_uuid.u,
                        .att_flags = BLE_GATT_CHR_F_READ,
                        .access_cb = gatt_svr_desc_access_wrapper,
                        .arg = WRAP_BYTE_ARRAY(gatt_svr_measurement_state_desc)
                    },
                    { 0 }
                }
            }, {
                0, /* No more characteristics in this service. */
            }
        }
    },

    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_info_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                //Manufacturer name
                .uuid = &gatt_svr_chr_manufacturer_uuid.u,
                .access_cb = gatt_svr_chr_constant_wrapper,
                .flags = BLE_GATT_CHR_F_READ,
                .arg = &my_ble_manufacturer_name_str
            },
            {
                //Model string
                .uuid = &gatt_svr_chr_model_uuid.u,
                .access_cb = gatt_svr_chr_constant_wrapper,
                .flags = BLE_GATT_CHR_F_READ,
                .arg = &my_ble_model_str
            },
            {
                //Serial number
                .uuid = &gatt_svr_chr_sn_uuid.u,
                .access_cb = gatt_svr_chr_constant_wrapper,
                .flags = BLE_GATT_CHR_F_READ,
                .arg = &my_ble_serial_number_str
            },
            {
                //Firmware revision
                .uuid = &gatt_svr_chr_fw_uuid.u,
                .access_cb = gatt_svr_chr_constant_wrapper,
                .flags = BLE_GATT_CHR_F_READ,
                .arg = &my_ble_fw_revision_str
            },
            {
                //Firmware revision
                .uuid = &gatt_svr_chr_pcb_uuid.u,
                .access_cb = gatt_svr_chr_constant_wrapper,
                .flags = BLE_GATT_CHR_F_READ,
                .arg = &my_ble_pcb_revision_str
            },
            { 0 }
        }
    },

    {
        0, /* No more services. */
    },
};

/*static int
gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    
    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}*/

/**
 * @brief BLE event handlers and their thread-safe wrappers
 * 
 */
///@{
static int
gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    const ble_uuid_t *uuid;
    int rc;

    uuid = ctxt->chr->uuid;

    /* Determine which characteristic is being accessed by examining its UUID. */

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_measured_conc_uuid.u) == 0) {
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);

        rc = os_mbuf_append(ctxt->om, &gatt_svr_measured_val, sizeof(gatt_svr_measured_val));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_measurement_state_uuid.u) == 0) {
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
        
        rc = os_mbuf_append(ctxt->om, &gatt_svr_measurement_state, sizeof(gatt_svr_measurement_state));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    /* Unknown characteristic; the nimble stack should not have called this
     * function.
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_access_wrapper(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    BaseType_t taken = xSemaphoreTakeRecursive(gatts_mutex, pdMS_TO_TICKS(500));
    int ret = 0;
    if (taken == pdTRUE)
    {
        ret = gatt_svr_chr_access(conn_handle, attr_handle, ctxt, arg);
        xSemaphoreGiveRecursive(gatts_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to acquire characteristic access mutex");
    }
    return ret;
}

static int
gatt_svr_chr_constant(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    const char na[] = "N/A";
    int rc;
    const char* str = (const char*)arg;

    if (!str) str = na;
    rc = os_mbuf_append(ctxt->om, str, strnlen(str, CONST_STRING_MAX_LEN) * sizeof(char));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int
gatt_svr_chr_constant_wrapper(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    BaseType_t taken = xSemaphoreTakeRecursive(gatts_mutex, pdMS_TO_TICKS(500));
    int ret = 0;
    if (taken == pdTRUE)
    {
        ret = gatt_svr_chr_constant(conn_handle, attr_handle, ctxt, *(char**)arg);
        xSemaphoreGiveRecursive(gatts_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to acquire const characteristic access mutex");
    }
    return ret;
}

static int
gatt_svr_desc_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    array_t* a = (array_t*)arg;
    assert(a);
    assert(a->items);
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC)
    {
        int rc = os_mbuf_append(ctxt->om, a->items, a->len);
        ESP_LOGD(TAG, "Descriptor access: %p, len = %u", a->items, a->len);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC)
    {
        if (ctxt->dsc->uuid == &(gatt_svr_desc_es_trigger_uuid.u))
        {
            uint16_t copied;
            int rc;
            if ((rc = ble_hs_mbuf_to_flat(ctxt->om, a->items, a->len, &copied)) != BLE_ERR_SUCCESS) return rc;
            ESP_LOGD(TAG, "Trigger write: %c", a->items[0]);
            return gatt_svc_es_trigger_cb(a->items[0]);
        }
        return BLE_ERR_CMD_DISALLOWED;
    }
    return BLE_ERR_CMD_DISALLOWED;
}

static int
gatt_svr_desc_access_wrapper(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    BaseType_t taken = xSemaphoreTakeRecursive(gatts_mutex, pdMS_TO_TICKS(500));
    int ret = 0;
    if (taken == pdTRUE)
    {
        ret = gatt_svr_desc_access(conn_handle, attr_handle, ctxt, arg);
        xSemaphoreGiveRecursive(gatts_mutex);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to acquire descriptor access mutex");
    }
    return ret;
}
///@}

/// @brief GATT server "service/characteristic/descriptor registered by NimBLE stack" callback, used in my_ble.c, declared in my_ble.h
/// @param ctxt 
/// @param arg 
void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

/// @brief Initialize both GATT server and the (required) underlying GAP layer
/// @return 0 if successful, a NimBLE error code otherwise
int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Count_cfg failed: %i", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Add_svcs failed: %i", rc);
        return rc;
    }

    return 0;
}
