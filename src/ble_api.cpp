/**
 * @file        ble_api.cpp
 * @brief       NimBLE implementation of BlePeripheral and BleCentral
 */

#include "ble_api.hpp"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char* TAG = "ble_api";

extern "C" void ble_store_config_init(void);

/* ── Host role lock ──────────────────────────────────────────────────────── */

typedef enum {
    eBleHostIdle       = 0,
    eBleHostPeripheral = 1,
    eBleHostCentral    = 2,
} ble_host_role_t;

static ble_host_role_t s_host_role  = eBleHostIdle;
static volatile bool   s_host_ready = false;
static bool            s_nvs_ready  = false;
static uint8_t         s_own_addr_type = BLE_OWN_ADDR_PUBLIC;

/** Cross-task lock: app task <-> NimBLE host task. */
static portMUX_TYPE s_ble_mux = portMUX_INITIALIZER_UNLOCKED;

static constexpr size_t kBleCbBufBytes = 128U;

static BlePeripheral* s_peri = nullptr;
static BleCentral*    s_cent = nullptr;

static ble_conn_cb_t s_peri_conn_cb = nullptr;
static ble_data_cb_t s_peri_rx_cb   = nullptr;

static ble_conn_cb_t s_cent_conn_cb   = nullptr;
static ble_data_cb_t s_cent_tx_cb     = nullptr;
static ble_data_cb_t s_cent_status_cb = nullptr;

/* NimBLE UUID objects (type + 16 bytes) */
static ble_uuid128_t s_svc_uuid = {};
static ble_uuid128_t s_rx_uuid  = {};
static ble_uuid128_t s_tx_uuid  = {};
static ble_uuid128_t s_st_uuid  = {};

/* Peripheral */
static uint16_t s_peri_conn       = BLE_HS_CONN_HANDLE_NONE;
static bool     s_peri_tx_sub     = false;
static bool     s_peri_status_sub = false;
static uint16_t s_peri_rx_handle  = 0;
static uint16_t s_peri_tx_handle  = 0;
static uint16_t s_peri_st_handle  = 0;

static struct ble_gatt_svc_def s_gatt_svcs[2];
static struct ble_gatt_chr_def s_gatt_chrs[4];

/* Central */
static uint16_t s_cent_conn       = BLE_HS_CONN_HANDLE_NONE;
static bool     s_cent_scanning   = false;
static bool     s_cent_connecting = false;
static bool     s_cent_ready      = false;
static uint16_t s_cent_rx_handle  = 0;
static uint16_t s_cent_tx_handle  = 0;
static uint16_t s_cent_tx_cccd    = 0;
static uint16_t s_cent_st_handle  = 0;
static uint16_t s_cent_st_cccd    = 0;
static uint16_t s_cent_svc_start  = 0;
static uint16_t s_cent_svc_end    = 0;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void loadUuid(ble_uuid128_t* dst, const ble_uuid_bytes_t& src)
{
    dst->u.type = BLE_UUID_TYPE_128;
    memcpy(dst->value, src.bytes, 16);
}

static void loadUuidsFromCfg(const ble_cfg_t& cfg)
{
    loadUuid(&s_svc_uuid, cfg.service_uuid);
    loadUuid(&s_rx_uuid, cfg.rx_uuid);
    loadUuid(&s_tx_uuid, cfg.tx_uuid);
    loadUuid(&s_st_uuid, cfg.status_uuid);
}

static esp_err_t ensureNvs(void)
{
    if (s_nvs_ready) {
        return ESP_OK;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs init");
    s_nvs_ready = true;
    return ESP_OK;
}

static void onSyncCommon(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer_addr rc=%d", rc);
        return;
    }
    s_host_ready = true;
    ESP_LOGI(TAG, "NimBLE sync ok addr_type=%u", (unsigned)s_own_addr_type);
}

static void onResetCommon(int reason)
{
    s_host_ready = false;
    ESP_LOGW(TAG, "NimBLE reset reason=%d", reason);
}

static void hostTask(void* /*param*/)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static bool waitHostReady(void)
{
    for (int i = 0; i < 50; ++i) {
        if (s_host_ready) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return s_host_ready;
}

static esp_err_t hostStop(void)
{
    if (s_host_role == eBleHostIdle) {
        return ESP_OK;
    }
    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_stop rc=%d — skip deinit", rc);
        return ESP_FAIL;
    }
    nimble_port_deinit();
    portENTER_CRITICAL(&s_ble_mux);
    s_host_role       = eBleHostIdle;
    s_host_ready      = false;
    s_peri            = nullptr;
    s_cent            = nullptr;
    s_peri_conn       = BLE_HS_CONN_HANDLE_NONE;
    s_cent_conn       = BLE_HS_CONN_HANDLE_NONE;
    s_cent_ready      = false;
    s_cent_scanning   = false;
    s_cent_connecting = false;
    portEXIT_CRITICAL(&s_ble_mux);
    return ESP_OK;
}

static esp_err_t notifyCustom(uint16_t conn, uint16_t handle,
                              const uint8_t* data, size_t len, bool subscribed)
{
    if (conn == BLE_HS_CONN_HANDLE_NONE || handle == 0 || !subscribed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || len == 0U || len > kBleMaxPayloadBytes) {
        return ESP_ERR_INVALID_ARG;
    }
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
    if (om == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    int rc = ble_gatts_notify_custom(conn, handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "notify_custom rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ── Peripheral GATT / GAP ───────────────────────────────────────────────── */

static int periGapEvent(struct ble_gap_event* event, void* arg);

static int periAccess(uint16_t /*conn_handle*/, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt* ctxt, void* /*arg*/)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        if (attr_handle != s_peri_rx_handle) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0U || om_len > kBleCbBufBytes) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        uint8_t buf[kBleCbBufBytes];
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ble_data_cb_t cb = nullptr;
        portENTER_CRITICAL(&s_ble_mux);
        cb = s_peri_rx_cb;
        portEXIT_CRITICAL(&s_ble_mux);
        if (cb != nullptr) {
            cb(buf, copied);
        }
        return 0;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static esp_err_t periRegisterGatt(void)
{
    memset(s_gatt_chrs, 0, sizeof(s_gatt_chrs));
    s_gatt_chrs[0].uuid       = &s_rx_uuid.u;
    s_gatt_chrs[0].access_cb  = periAccess;
    s_gatt_chrs[0].flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;
    s_gatt_chrs[0].val_handle = &s_peri_rx_handle;

    s_gatt_chrs[1].uuid       = &s_tx_uuid.u;
    s_gatt_chrs[1].access_cb  = periAccess;
    s_gatt_chrs[1].flags      = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ;
    s_gatt_chrs[1].val_handle = &s_peri_tx_handle;

    s_gatt_chrs[2].uuid       = &s_st_uuid.u;
    s_gatt_chrs[2].access_cb  = periAccess;
    s_gatt_chrs[2].flags      = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ;
    s_gatt_chrs[2].val_handle = &s_peri_st_handle;

    memset(s_gatt_svcs, 0, sizeof(s_gatt_svcs));
    s_gatt_svcs[0].type            = BLE_GATT_SVC_TYPE_PRIMARY;
    s_gatt_svcs[0].uuid            = &s_svc_uuid.u;
    s_gatt_svcs[0].characteristics = s_gatt_chrs;

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t periAdvertiseNow(const ble_cfg_t& cfg)
{
    if (!s_host_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const char* name =
        (cfg.device_name != nullptr) ? cfg.device_name : kBleDefaultDeviceName;

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<const uint8_t*>(name);
    fields.name_len = static_cast<uint8_t>(strlen(name));
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return ESP_FAIL;
    }

    /* Put 128-bit service UUID in scan response (ADV often too small). */
    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.uuids128 = &s_svc_uuid;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGW(TAG, "adv_rsp_set_fields rc=%d", rc);
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER,
                           &adv_params, periGapEvent, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "advertising as \"%s\"", name);
    return ESP_OK;
}

static int periGapEvent(struct ble_gap_event* event, void* /*arg*/)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            portENTER_CRITICAL(&s_ble_mux);
            s_peri_conn = event->connect.conn_handle;
            ble_conn_cb_t cb = s_peri_conn_cb;
            uint16_t handle = s_peri_conn;
            portEXIT_CRITICAL(&s_ble_mux);
            ESP_LOGI(TAG, "peri connected handle=%u", (unsigned)handle);
            if (cb != nullptr) {
                cb(true, handle);
            }
        } else {
            ESP_LOGW(TAG, "peri connect fail status=%d", event->connect.status);
            if (s_peri != nullptr && s_peri->getConfig().auto_advertise) {
                (void)periAdvertiseNow(s_peri->getConfig());
            }
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGI(TAG, "peri disconnect reason=%d", event->disconnect.reason);
        portENTER_CRITICAL(&s_ble_mux);
        s_peri_conn = BLE_HS_CONN_HANDLE_NONE;
        s_peri_tx_sub = false;
        s_peri_status_sub = false;
        ble_conn_cb_t cb = s_peri_conn_cb;
        portEXIT_CRITICAL(&s_ble_mux);
        if (cb != nullptr) {
            cb(false, event->disconnect.conn.conn_handle);
        }
        if (s_peri != nullptr && s_peri->getConfig().auto_advertise) {
            (void)periAdvertiseNow(s_peri->getConfig());
        }
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        portENTER_CRITICAL(&s_ble_mux);
        if (event->subscribe.attr_handle == s_peri_tx_handle) {
            s_peri_tx_sub = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "TX CCCD notify=%d", (int)s_peri_tx_sub);
        } else if (event->subscribe.attr_handle == s_peri_st_handle) {
            s_peri_status_sub = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "status CCCD notify=%d", (int)s_peri_status_sub);
        }
        portEXIT_CRITICAL(&s_ble_mux);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (s_peri != nullptr && s_peri->getConfig().auto_advertise &&
            s_peri_conn == BLE_HS_CONN_HANDLE_NONE) {
            (void)periAdvertiseNow(s_peri->getConfig());
        }
        return 0;

    default:
        return 0;
    }
}

/* ── Central GAP / GATT ──────────────────────────────────────────────────── */

static int centGapEvent(struct ble_gap_event* event, void* arg);

static bool advHasServiceUuid(const struct ble_hs_adv_fields* fields)
{
    for (uint8_t i = 0; i < fields->num_uuids128; ++i) {
        if (ble_uuid_cmp(&fields->uuids128[i].u, &s_svc_uuid.u) == 0) {
            return true;
        }
    }
    return false;
}

static bool advNameMatches(const struct ble_hs_adv_fields* fields,
                           const char* filter)
{
    if (filter == nullptr || filter[0] == '\0') {
        return true;
    }
    if (fields->name == nullptr || fields->name_len == 0) {
        return false;
    }
    const size_t flen = strlen(filter);
    if (flen != fields->name_len) {
        return false;
    }
    return memcmp(fields->name, filter, flen) == 0;
}

static int centOnCccdWrite(uint16_t /*conn*/, const struct ble_gatt_error* error,
                           struct ble_gatt_attr* /*attr*/, void* /*arg*/)
{
    if (error->status != 0) {
        ESP_LOGW(TAG, "CCCD write status=%d", error->status);
    } else {
        ESP_LOGI(TAG, "CCCD write ok");
    }
    return 0;
}

static int centDiscDscs(uint16_t conn, const struct ble_gatt_error* error,
                        uint16_t chr_val_handle, const struct ble_gatt_dsc* dsc,
                        void* arg)
{
    const int stage = static_cast<int>(reinterpret_cast<uintptr_t>(arg));

    if (error->status == BLE_HS_EDONE) {
        if (stage == 1) {
            if (s_cent_st_handle != 0) {
                int rc = ble_gattc_disc_all_dscs(
                    conn, s_cent_st_handle, s_cent_svc_end, centDiscDscs,
                    reinterpret_cast<void*>(static_cast<uintptr_t>(2)));
                if (rc != 0) {
                    ESP_LOGE(TAG, "disc status dscs rc=%d", rc);
                    s_cent_ready = true;
                }
            } else {
                s_cent_ready = true;
                ESP_LOGI(TAG, "central GATT ready");
            }
        } else {
            portENTER_CRITICAL(&s_ble_mux);
            s_cent_ready = true;
            portEXIT_CRITICAL(&s_ble_mux);
            ESP_LOGI(TAG, "central GATT ready rx=%u tx=%u st=%u",
                     (unsigned)s_cent_rx_handle, (unsigned)s_cent_tx_handle,
                     (unsigned)s_cent_st_handle);
            const uint8_t en[2] = {1, 0};
            if (s_cent_st_cccd != 0) {
                int wrc = ble_gattc_write_flat(conn, s_cent_st_cccd, en,
                                               sizeof(en), centOnCccdWrite,
                                               nullptr);
                if (wrc != 0) {
                    ESP_LOGE(TAG, "status CCCD write rc=%d", wrc);
                }
            }
            if (s_cent_tx_cccd != 0) {
                int wrc = ble_gattc_write_flat(conn, s_cent_tx_cccd, en,
                                               sizeof(en), centOnCccdWrite,
                                               nullptr);
                if (wrc != 0) {
                    ESP_LOGE(TAG, "TX CCCD write rc=%d", wrc);
                }
            }
        }
        return 0;
    }
    if (error->status != 0) {
        return error->status;
    }
    if (dsc == nullptr) {
        return 0;
    }
    static const ble_uuid16_t kCccdUuid =
        BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);
    if (ble_uuid_cmp(&dsc->uuid.u, &kCccdUuid.u) == 0) {
        if (chr_val_handle == s_cent_tx_handle) {
            s_cent_tx_cccd = dsc->handle;
        } else if (chr_val_handle == s_cent_st_handle) {
            s_cent_st_cccd = dsc->handle;
        }
    }
    return 0;
}

static int centDiscChrs(uint16_t conn, const struct ble_gatt_error* error,
                        const struct ble_gatt_chr* chr, void* /*arg*/)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_cent_tx_handle != 0) {
            int rc = ble_gattc_disc_all_dscs(
                conn, s_cent_tx_handle, s_cent_svc_end, centDiscDscs,
                reinterpret_cast<void*>(static_cast<uintptr_t>(1)));
            if (rc != 0) {
                ESP_LOGE(TAG, "disc tx dscs rc=%d", rc);
                s_cent_ready = true;
            }
        } else {
            s_cent_ready = true;
        }
        return 0;
    }
    if (error->status != 0) {
        return error->status;
    }
    if (chr == nullptr) {
        return 0;
    }
    if (ble_uuid_cmp(&chr->uuid.u, &s_rx_uuid.u) == 0) {
        s_cent_rx_handle = chr->val_handle;
    } else if (ble_uuid_cmp(&chr->uuid.u, &s_tx_uuid.u) == 0) {
        s_cent_tx_handle = chr->val_handle;
    } else if (ble_uuid_cmp(&chr->uuid.u, &s_st_uuid.u) == 0) {
        s_cent_st_handle = chr->val_handle;
    }
    return 0;
}

static int centDiscSvcs(uint16_t conn, const struct ble_gatt_error* error,
                        const struct ble_gatt_svc* svc, void* /*arg*/)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_cent_svc_start == 0) {
            ESP_LOGW(TAG, "service UUID not found on peer");
            (void)ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }
        int rc = ble_gattc_disc_all_chrs(conn, s_cent_svc_start, s_cent_svc_end,
                                         centDiscChrs, nullptr);
        if (rc != 0) {
            ESP_LOGE(TAG, "disc_all_chrs rc=%d", rc);
        }
        return 0;
    }
    if (error->status != 0) {
        return error->status;
    }
    if (svc == nullptr) {
        return 0;
    }
    if (ble_uuid_cmp(&svc->uuid.u, &s_svc_uuid.u) == 0) {
        s_cent_svc_start = svc->start_handle;
        s_cent_svc_end   = svc->end_handle;
    }
    return 0;
}

static void centStartDiscovery(uint16_t conn)
{
    s_cent_ready     = false;
    s_cent_rx_handle = 0;
    s_cent_tx_handle = 0;
    s_cent_tx_cccd   = 0;
    s_cent_st_handle = 0;
    s_cent_st_cccd   = 0;
    s_cent_svc_start = 0;
    s_cent_svc_end   = 0;

    int rc = ble_gattc_disc_all_svcs(conn, centDiscSvcs, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "disc_all_svcs rc=%d", rc);
    }
}

static void centTryConnect(const ble_addr_t* addr)
{
    if (s_cent_connecting || s_cent_conn != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    s_cent_connecting = true;
    (void)ble_gap_disc_cancel();
    s_cent_scanning = false;

    uint8_t own_type = s_own_addr_type;
    int rc = ble_gap_connect(own_type, addr, 30000, nullptr, centGapEvent, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "gap_connect rc=%d", rc);
        s_cent_connecting = false;
    } else {
        ESP_LOGI(TAG, "connecting...");
    }
}

static bool centAdvMatches(const struct ble_hs_adv_fields* fields,
                           const ble_cfg_t& cfg)
{
    const bool has_uuid = advHasServiceUuid(fields);
    const bool name_ok  = advNameMatches(fields, cfg.peer_name_filter);

    if (has_uuid && name_ok) {
        return true;
    }
    /* Name-only match when filter is set and ADV has no 128-bit UUID list. */
    if (cfg.peer_name_filter != nullptr && name_ok && fields->num_uuids128 == 0) {
        return true;
    }
    return false;
}

static int centGapEvent(struct ble_gap_event* event, void* /*arg*/)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                    event->disc.length_data) != 0) {
            return 0;
        }
        if (s_cent == nullptr) {
            return 0;
        }
        if (!centAdvMatches(&fields, s_cent->getConfig())) {
            return 0;
        }
        ESP_LOGI(TAG, "scan hit; connecting");
        centTryConnect(&event->disc.addr);
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
        s_cent_scanning = false;
        ESP_LOGI(TAG, "scan complete reason=%d", event->disc_complete.reason);
        return 0;

    case BLE_GAP_EVENT_CONNECT: {
        s_cent_connecting = false;
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "cent connect fail status=%d", event->connect.status);
            return 0;
        }
        portENTER_CRITICAL(&s_ble_mux);
        s_cent_conn = event->connect.conn_handle;
        ble_conn_cb_t cb = s_cent_conn_cb;
        uint16_t handle = s_cent_conn;
        portEXIT_CRITICAL(&s_ble_mux);
        ESP_LOGI(TAG, "cent connected handle=%u", (unsigned)handle);
        if (cb != nullptr) {
            cb(true, handle);
        }
        centStartDiscovery(handle);
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT: {
        ESP_LOGI(TAG, "cent disconnect reason=%d", event->disconnect.reason);
        portENTER_CRITICAL(&s_ble_mux);
        s_cent_conn  = BLE_HS_CONN_HANDLE_NONE;
        s_cent_ready = false;
        ble_conn_cb_t cb = s_cent_conn_cb;
        portEXIT_CRITICAL(&s_ble_mux);
        if (cb != nullptr) {
            cb(false, event->disconnect.conn.conn_handle);
        }
        return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
        const uint16_t attr = event->notify_rx.attr_handle;
        const uint16_t len  = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len == 0U || len > kBleCbBufBytes) {
            return 0;
        }
        uint8_t buf[kBleCbBufBytes];
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &copied) != 0) {
            return 0;
        }
        ble_data_cb_t tx_cb = nullptr;
        ble_data_cb_t st_cb = nullptr;
        uint16_t tx_h = 0;
        uint16_t st_h = 0;
        portENTER_CRITICAL(&s_ble_mux);
        tx_cb = s_cent_tx_cb;
        st_cb = s_cent_status_cb;
        tx_h  = s_cent_tx_handle;
        st_h  = s_cent_st_handle;
        portEXIT_CRITICAL(&s_ble_mux);
        if (attr == tx_h && tx_cb != nullptr) {
            tx_cb(buf, copied);
        } else if (attr == st_h && st_cb != nullptr) {
            st_cb(buf, copied);
        }
        return 0;
    }

    default:
        return 0;
    }
}

/* ── BlePeripheral ───────────────────────────────────────────────────────── */

BlePeripheral::BlePeripheral(const ble_cfg_t& config)
    : m_config(config)
{
}

BlePeripheral::~BlePeripheral()
{
    (void)deinit();
}

esp_err_t BlePeripheral::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "peripheral already inited");
        return ESP_OK;
    }
    if (s_host_role != eBleHostIdle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (m_config.device_name == nullptr) {
        m_config.device_name = kBleDefaultDeviceName;
    }

    loadUuidsFromCfg(m_config);
    s_peri_conn_cb    = m_conn_cb;
    s_peri_rx_cb      = m_rx_cb;
    s_peri            = this;
    s_peri_tx_sub     = false;
    s_peri_status_sub = false;
    s_peri_rx_handle  = 0;
    s_peri_tx_handle  = 0;
    s_peri_st_handle  = 0;

    ESP_RETURN_ON_ERROR(ensureNvs(), TAG, "nvs");

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        s_peri = nullptr;
        ESP_LOGE(TAG, "nimble_port_init %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb        = onResetCommon;
    ble_hs_cfg.sync_cb         = onSyncCommon;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    err = periRegisterGatt();
    if (err != ESP_OK) {
        nimble_port_deinit();
        s_peri = nullptr;
        return err;
    }

    (void)ble_svc_gap_device_name_set(m_config.device_name);

    s_host_role = eBleHostPeripheral;
    nimble_port_freertos_init(hostTask);
    if (!waitHostReady()) {
        ESP_LOGE(TAG, "NimBLE sync timeout");
        (void)nimble_port_stop();
        nimble_port_deinit();
        s_host_role = eBleHostIdle;
        s_peri = nullptr;
        return ESP_ERR_TIMEOUT;
    }

    m_inited = true;
    if (m_config.auto_advertise) {
        (void)startAdvertising();
    }
    return ESP_OK;
}

esp_err_t BlePeripheral::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }
    (void)stopAdvertising();
    if (s_peri_conn != BLE_HS_CONN_HANDLE_NONE) {
        (void)ble_gap_terminate(s_peri_conn, BLE_ERR_REM_USER_CONN_TERM);
        s_peri_conn = BLE_HS_CONN_HANDLE_NONE;
    }
    s_peri         = nullptr;
    s_peri_conn_cb = nullptr;
    s_peri_rx_cb   = nullptr;
    m_inited       = false;
    return hostStop();
}

esp_err_t BlePeripheral::startAdvertising()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_peri_conn != BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ble_gap_adv_active()) {
        return ESP_OK;
    }
    return periAdvertiseNow(m_config);
}

esp_err_t BlePeripheral::stopAdvertising()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ble_gap_adv_active()) {
        return ESP_OK;
    }
    int rc = ble_gap_adv_stop();
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t BlePeripheral::notifyTx(const uint8_t* data, size_t len)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_ble_mux);
    const uint16_t conn = s_peri_conn;
    const uint16_t handle = s_peri_tx_handle;
    const bool sub = s_peri_tx_sub;
    portEXIT_CRITICAL(&s_ble_mux);
    return notifyCustom(conn, handle, data, len, sub);
}

esp_err_t BlePeripheral::notifyStatus(const uint8_t* data, size_t len)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_ble_mux);
    const uint16_t conn = s_peri_conn;
    const uint16_t handle = s_peri_st_handle;
    const bool sub = s_peri_status_sub;
    portEXIT_CRITICAL(&s_ble_mux);
    return notifyCustom(conn, handle, data, len, sub);
}

esp_err_t BlePeripheral::notifyStatus(const char* utf8)
{
    if (utf8 == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return notifyStatus(reinterpret_cast<const uint8_t*>(utf8), strlen(utf8));
}

void BlePeripheral::setConnCallback(ble_conn_cb_t cb)
{
    m_conn_cb = cb;
    portENTER_CRITICAL(&s_ble_mux);
    s_peri_conn_cb = cb;
    portEXIT_CRITICAL(&s_ble_mux);
}

void BlePeripheral::setRxCallback(ble_data_cb_t cb)
{
    m_rx_cb = cb;
    portENTER_CRITICAL(&s_ble_mux);
    s_peri_rx_cb = cb;
    portEXIT_CRITICAL(&s_ble_mux);
}

bool BlePeripheral::isConnected() const
{
    if (!m_inited) {
        return false;
    }
    portENTER_CRITICAL(&s_ble_mux);
    const bool connected = (s_peri_conn != BLE_HS_CONN_HANDLE_NONE);
    portEXIT_CRITICAL(&s_ble_mux);
    return connected;
}

bool BlePeripheral::isInited() const
{
    return m_inited;
}

const ble_cfg_t& BlePeripheral::getConfig() const
{
    return m_config;
}

/* ── BleCentral ──────────────────────────────────────────────────────────── */

BleCentral::BleCentral(const ble_cfg_t& config)
    : m_config(config)
{
}

BleCentral::~BleCentral()
{
    (void)deinit();
}

esp_err_t BleCentral::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "central already inited");
        return ESP_OK;
    }
    if (s_host_role != eBleHostIdle) {
        return ESP_ERR_INVALID_STATE;
    }

    loadUuidsFromCfg(m_config);
    s_cent_conn_cb   = m_conn_cb;
    s_cent_tx_cb     = m_tx_cb;
    s_cent_status_cb = m_status_cb;
    s_cent           = this;

    ESP_RETURN_ON_ERROR(ensureNvs(), TAG, "nvs");

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        s_cent = nullptr;
        ESP_LOGE(TAG, "nimble_port_init %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.reset_cb        = onResetCommon;
    ble_hs_cfg.sync_cb         = onSyncCommon;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    s_host_role = eBleHostCentral;
    nimble_port_freertos_init(hostTask);
    if (!waitHostReady()) {
        ESP_LOGE(TAG, "NimBLE sync timeout");
        (void)nimble_port_stop();
        nimble_port_deinit();
        s_host_role = eBleHostIdle;
        s_cent = nullptr;
        return ESP_ERR_TIMEOUT;
    }

    m_inited = true;
    return ESP_OK;
}

esp_err_t BleCentral::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }
    (void)stopScan();
    (void)disconnect();
    s_cent           = nullptr;
    s_cent_conn_cb   = nullptr;
    s_cent_tx_cb     = nullptr;
    s_cent_status_cb = nullptr;
    m_inited         = false;
    return hostStop();
}

esp_err_t BleCentral::startScan()
{
    if (!m_inited || !s_host_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cent_conn != BLE_HS_CONN_HANDLE_NONE || s_cent_connecting) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cent_scanning) {
        return ESP_OK;
    }

    struct ble_gap_disc_params disc;
    memset(&disc, 0, sizeof(disc));
    disc.filter_duplicates = 1;
    disc.itvl   = BLE_GAP_SCAN_FAST_INTERVAL_MIN;
    disc.window = BLE_GAP_SCAN_FAST_WINDOW;

    int32_t duration = BLE_HS_FOREVER;
    if (m_config.scan_ms > 0U) {
        duration = static_cast<int32_t>(m_config.scan_ms);
    }

    int rc = ble_gap_disc(s_own_addr_type, duration, &disc, centGapEvent, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "gap_disc rc=%d", rc);
        return ESP_FAIL;
    }
    s_cent_scanning = true;
    ESP_LOGI(TAG, "scanning (%u ms)", (unsigned)m_config.scan_ms);
    return ESP_OK;
}

esp_err_t BleCentral::stopScan()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_cent_scanning) {
        return ESP_OK;
    }
    int rc = ble_gap_disc_cancel();
    s_cent_scanning = false;
    return (rc == 0 || rc == BLE_HS_EALREADY) ? ESP_OK : ESP_FAIL;
}

esp_err_t BleCentral::disconnect()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cent_conn == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_OK;
    }
    int rc = ble_gap_terminate(s_cent_conn, BLE_ERR_REM_USER_CONN_TERM);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t BleCentral::writeRx(const uint8_t* data, size_t len)
{
    if (!m_inited || !s_cent_ready || s_cent_rx_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || len == 0U || len > kBleMaxPayloadBytes) {
        return ESP_ERR_INVALID_ARG;
    }
    int rc = ble_gattc_write_no_rsp_flat(s_cent_conn, s_cent_rx_handle, data, len);
    if (rc != 0) {
        ESP_LOGE(TAG, "write_rx rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t BleCentral::writeRx(const char* utf8)
{
    if (utf8 == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return writeRx(reinterpret_cast<const uint8_t*>(utf8), strlen(utf8));
}

esp_err_t BleCentral::subscribeTx(bool enable)
{
    if (!m_inited || s_cent_tx_cccd == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t val[2] = { static_cast<uint8_t>(enable ? 1 : 0), 0 };
    int rc = ble_gattc_write_flat(s_cent_conn, s_cent_tx_cccd, val, sizeof(val),
                                  centOnCccdWrite, nullptr);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t BleCentral::subscribeStatus(bool enable)
{
    if (!m_inited || s_cent_st_cccd == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t val[2] = { static_cast<uint8_t>(enable ? 1 : 0), 0 };
    int rc = ble_gattc_write_flat(s_cent_conn, s_cent_st_cccd, val, sizeof(val),
                                  centOnCccdWrite, nullptr);
    return (rc == 0) ? ESP_OK : ESP_FAIL;
}

void BleCentral::setConnCallback(ble_conn_cb_t cb)
{
    m_conn_cb = cb;
    portENTER_CRITICAL(&s_ble_mux);
    s_cent_conn_cb = cb;
    portEXIT_CRITICAL(&s_ble_mux);
}

void BleCentral::setTxNotifyCallback(ble_data_cb_t cb)
{
    m_tx_cb = cb;
    portENTER_CRITICAL(&s_ble_mux);
    s_cent_tx_cb = cb;
    portEXIT_CRITICAL(&s_ble_mux);
}

void BleCentral::setStatusNotifyCallback(ble_data_cb_t cb)
{
    m_status_cb = cb;
    portENTER_CRITICAL(&s_ble_mux);
    s_cent_status_cb = cb;
    portEXIT_CRITICAL(&s_ble_mux);
}

bool BleCentral::isConnected() const
{
    if (!m_inited) {
        return false;
    }
    portENTER_CRITICAL(&s_ble_mux);
    const bool connected = (s_cent_conn != BLE_HS_CONN_HANDLE_NONE);
    portEXIT_CRITICAL(&s_ble_mux);
    return connected;
}

bool BleCentral::isInited() const
{
    return m_inited;
}

bool BleCentral::isReady() const
{
    if (!m_inited) {
        return false;
    }
    portENTER_CRITICAL(&s_ble_mux);
    const bool ready = s_cent_ready;
    portEXIT_CRITICAL(&s_ble_mux);
    return ready;
}

const ble_cfg_t& BleCentral::getConfig() const
{
    return m_config;
}
