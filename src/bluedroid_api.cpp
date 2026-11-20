/**
 * @file        bluedroid_api.cpp
 * @brief       Bluedroid implementation of BluedroidPeripheral and BluedroidCentral
 */

#include "sdkconfig.h"

#if CONFIG_BT_BLUEDROID_ENABLED

#include "bluedroid_api.hpp"

#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_gattc_api.h"

static const char* TAG = "bluedroid_api";

/* ── Host role lock ──────────────────────────────────────────────────────── */

typedef enum {
    eBdHostIdle       = 0,
    eBdHostPeripheral = 1,
    eBdHostCentral    = 2,
} bd_host_role_t;

static bd_host_role_t s_host_role   = eBdHostIdle;
static bool           s_stack_ready = false;
static bool           s_nvs_ready   = false;
static portMUX_TYPE   s_bd_mux      = portMUX_INITIALIZER_UNLOCKED;

static BluedroidPeripheral* s_peri = nullptr;
static BluedroidCentral*    s_cent = nullptr;

static bluedroid_conn_cb_t s_peri_conn_cb = nullptr;
static bluedroid_data_cb_t s_peri_rx_cb   = nullptr;

static bluedroid_conn_cb_t s_cent_conn_cb   = nullptr;
static bluedroid_data_cb_t s_cent_tx_cb     = nullptr;
static bluedroid_data_cb_t s_cent_status_cb = nullptr;

static bluedroid_auth_cb_t    s_auth_cb    = nullptr;
static bluedroid_passkey_cb_t s_passkey_cb = nullptr;
static esp_bd_addr_t          s_peri_peer_bda = {};
static bool                   s_link_encrypted = false;
static bool                   s_link_bonded    = false;
static constexpr uint8_t      kBdSmMaxKeySize  = 16U;

static constexpr uint16_t kBdInvalidHandle = 0xFFFFU;
static constexpr uint16_t kBdInvalidConn   = 0xFFFFU;
static constexpr uint16_t kBdPreferredMtu  = 517U;

static constexpr TickType_t kPeriAdvRestartDelayTicks = pdMS_TO_TICKS(300);
static constexpr TickType_t kPeriAdvRetryDelayTicks   = pdMS_TO_TICKS(500);
static constexpr int        kPeriAdvMaxRetries        = 3;

/* Peripheral GATT state */
static uint16_t s_peri_gatts_if   = ESP_GATT_IF_NONE;
static uint16_t s_peri_app_id     = 0x55U;
static uint16_t s_peri_conn_id    = kBdInvalidConn;
static uint16_t s_peri_service_h  = kBdInvalidHandle;
static uint16_t s_peri_rx_h       = kBdInvalidHandle;
static uint16_t s_peri_tx_h       = kBdInvalidHandle;
static uint16_t s_peri_st_h       = kBdInvalidHandle;
static uint16_t s_peri_tx_cccd_h  = kBdInvalidHandle;
static uint16_t s_peri_st_cccd_h  = kBdInvalidHandle;
static bool     s_peri_tx_sub     = false;
static bool     s_peri_st_sub     = false;
static bool     s_peri_service_started = false;
static bool     s_peri_adv_ready  = false;
static bool     s_peri_scan_rsp_ready = false;
static bool     s_peri_adv_start_pending = false;
static uint8_t  s_peri_char_step  = 0;

static esp_bt_uuid_t s_svc_uuid = {};
static esp_bt_uuid_t s_rx_uuid  = {};
static esp_bt_uuid_t s_tx_uuid  = {};
static esp_bt_uuid_t s_st_uuid  = {};

static TimerHandle_t s_peri_adv_timer        = nullptr;
static bool          s_peri_adv_retry_armed  = false;
static int           s_peri_adv_retry_count  = 0;

/* Central GATT state */
static uint16_t s_cent_gattc_if   = ESP_GATT_IF_NONE;
static uint16_t s_cent_app_id     = 0x56U;
static uint16_t s_cent_conn_id    = kBdInvalidConn;
static bool     s_cent_scanning   = false;
static bool     s_cent_connecting = false;
static bool     s_cent_ready      = false;
static uint16_t s_cent_rx_h       = kBdInvalidHandle;
static uint16_t s_cent_tx_h       = kBdInvalidHandle;
static uint16_t s_cent_st_h       = kBdInvalidHandle;
static uint16_t s_cent_tx_cccd    = kBdInvalidHandle;
static uint16_t s_cent_st_cccd    = kBdInvalidHandle;
static esp_bd_addr_t s_cent_peer_bda = {};
static esp_ble_addr_type_t s_cent_peer_addr_type = BLE_ADDR_TYPE_PUBLIC;

static SemaphoreHandle_t s_init_sem = nullptr;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void loadUuid(esp_bt_uuid_t* dst, const bluedroid_uuid_bytes_t& src)
{
    dst->len = ESP_UUID_LEN_128;
    memcpy(dst->uuid.uuid128, src.bytes, 16);
}

static bool uuidEqual(const esp_bt_uuid_t& a, const esp_bt_uuid_t& b)
{
    if (a.len != b.len) {
        return false;
    }
    if (a.len == ESP_UUID_LEN_16) {
        return a.uuid.uuid16 == b.uuid.uuid16;
    }
    if (a.len == ESP_UUID_LEN_32) {
        return a.uuid.uuid32 == b.uuid.uuid32;
    }
    return memcmp(a.uuid.uuid128, b.uuid.uuid128, 16) == 0;
}

static esp_err_t ensureNvs(void)
{
    if (s_nvs_ready) {
        return ESP_OK;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init");
    s_nvs_ready = true;
    return ESP_OK;
}

static esp_err_t startControllerAndBluedroid(void)
{
    if (s_stack_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ensureNvs(), TAG, "ensureNvs");

    /* ESP32-S3 has no classic BT — free that controller RAM. */
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "classic mem_release: %s", esp_err_to_name(err));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "controller already inited");
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "esp_bt_controller_init");
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "controller already enabled");
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "esp_bt_controller_enable");
    }

    err = esp_bluedroid_init();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "bluedroid already inited");
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "esp_bluedroid_init");
    }

    err = esp_bluedroid_enable();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "bluedroid already enabled");
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "esp_bluedroid_enable");
    }

    (void)esp_ble_gatt_set_local_mtu(kBdPreferredMtu);

    s_stack_ready = true;
    return ESP_OK;
}

static esp_err_t stopControllerAndBluedroid(void)
{
    if (!s_stack_ready) {
        return ESP_OK;
    }

    esp_err_t err = esp_bluedroid_disable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "bluedroid_disable: %s", esp_err_to_name(err));
    }
    err = esp_bluedroid_deinit();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "bluedroid_deinit: %s", esp_err_to_name(err));
    }
    err = esp_bt_controller_disable();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "controller_disable: %s", esp_err_to_name(err));
    }
    err = esp_bt_controller_deinit();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "controller_deinit: %s", esp_err_to_name(err));
    }

    s_stack_ready = false;
    return ESP_OK;
}

static void signalInitDone(void)
{
    if (s_init_sem != nullptr) {
        (void)xSemaphoreGive(s_init_sem);
    }
}

/* ── Peripheral advertising ──────────────────────────────────────────────── */

/**
 * @brief Start ADV after both ADV and SCAN_RSP raw payloads are accepted.
 * @details
 *   Espressif: iOS Settings often hides peripherals without a service UUID in
 *   the primary ADV. Name + 128-bit UUID do not fit in 31 bytes, so put UUID
 *   in ADV and the complete local name in SCAN_RSP (IDF / BLE common pattern).
 *   Do not clear SCAN_RSP to empty — ADV_TYPE_IND is scannable; active scanners
 *   (many phones) request SCAN_RSP and may drop the device if it is missing.
 */
static esp_err_t periStartAdvParams(void)
{
    esp_ble_adv_params_t params = {};
    params.adv_int_min       = 0x20; /* 20 ms */
    params.adv_int_max       = 0x40; /* 40 ms */
    params.adv_type          = ADV_TYPE_IND;
    params.own_addr_type     = BLE_ADDR_TYPE_PUBLIC;
    params.channel_map       = ADV_CHNL_ALL;
    params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

    const esp_err_t err = esp_ble_gap_start_advertising(&params);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start_advertising: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

static void periTryStartAdvAfterData(void)
{
    if (!s_peri_adv_start_pending) {
        return;
    }
    if (!s_peri_adv_ready || !s_peri_scan_rsp_ready) {
        return;
    }
    if (s_peri_conn_id != kBdInvalidConn) {
        s_peri_adv_start_pending = false;
        return;
    }

    s_peri_adv_start_pending = false;
    if (periStartAdvParams() != ESP_OK) {
        return;
    }

    const char* name = kBluedroidDefaultDeviceName;
    if (s_peri != nullptr && s_peri->getConfig().device_name != nullptr) {
        name = s_peri->getConfig().device_name;
    }
    ESP_LOGI(TAG, "advertising as \"%s\" (UUID in ADV, name in SCAN_RSP)", name);
}

static esp_err_t periSetAdvData(const bluedroid_cfg_t& cfg)
{
    const char* name =
        (cfg.device_name != nullptr) ? cfg.device_name : kBluedroidDefaultDeviceName;
    const size_t name_len = strlen(name);
    if (name_len == 0U) {
        ESP_LOGE(TAG, "empty device name");
        return ESP_ERR_INVALID_ARG;
    }

    /* Primary ADV: Flags (3) + Complete 128-bit service UUID list (18) = 21. */
    uint8_t adv[31];
    size_t  aoff = 0;
    adv[aoff++] = 2;
    adv[aoff++] = ESP_BLE_AD_TYPE_FLAG;
    adv[aoff++] = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
    adv[aoff++] = 17;
    adv[aoff++] = ESP_BLE_AD_TYPE_128SRV_CMPL;
    memcpy(&adv[aoff], s_svc_uuid.uuid.uuid128, 16);
    aoff += 16;

    /* SCAN_RSP: Complete Local Name (truncated if longer than 29 payload bytes). */
    uint8_t rsp[31];
    size_t  roff = 0;
    const size_t max_n = sizeof(rsp) - 2U;
    const size_t n = (name_len < max_n) ? name_len : max_n;
    rsp[roff++] = static_cast<uint8_t>(1U + n);
    rsp[roff++] = ESP_BLE_AD_TYPE_NAME_CMPL;
    memcpy(&rsp[roff], name, n);
    roff += n;
    if (n < name_len) {
        ESP_LOGW(TAG, "device name truncated in SCAN_RSP (%u -> %u)",
                 (unsigned)name_len, (unsigned)n);
    }

    s_peri_adv_ready = false;
    s_peri_scan_rsp_ready = false;

    esp_err_t err = esp_ble_gap_config_adv_data_raw(adv, static_cast<uint32_t>(aoff));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config_adv_data_raw: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_gap_config_scan_rsp_data_raw(rsp, static_cast<uint32_t>(roff));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "config_scan_rsp_data_raw: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ADV=flags+128UUID SCAN_RSP=name \"%.*s\"", (int)n, name);
    (void)esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    return ESP_OK;
}

static esp_err_t periAdvertiseNow(const bluedroid_cfg_t& cfg)
{
    if (!s_stack_ready || s_peri_gatts_if == ESP_GATT_IF_NONE || !s_peri_service_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_peri_conn_id != kBdInvalidConn) {
        return ESP_ERR_INVALID_STATE;
    }

    s_peri_adv_start_pending = true;
    ESP_RETURN_ON_ERROR(periSetAdvData(cfg), TAG, "periSetAdvData");
    /* Start runs from GAP SET_COMPLETE callbacks (periTryStartAdvAfterData). */
    periTryStartAdvAfterData();
    return ESP_OK;
}

static esp_err_t periRestartAdvertising(const bluedroid_cfg_t& cfg)
{
    if (s_peri_conn_id != kBdInvalidConn) {
        ESP_LOGW(TAG, "restart ADV skipped — still connected");
        return ESP_ERR_INVALID_STATE;
    }
    (void)esp_ble_gap_stop_advertising();
    return periAdvertiseNow(cfg);
}

static void periAdvTimerCb(TimerHandle_t /*timer*/)
{
    if (s_peri == nullptr || !s_peri->getConfig().auto_advertise) {
        s_peri_adv_retry_armed = false;
        s_peri_adv_retry_count = 0;
        return;
    }
    if (s_peri_conn_id != kBdInvalidConn) {
        s_peri_adv_retry_armed = false;
        s_peri_adv_retry_count = 0;
        return;
    }

    const esp_err_t err = periRestartAdvertising(s_peri->getConfig());
    if (err == ESP_OK) {
        s_peri_adv_retry_armed = false;
        s_peri_adv_retry_count = 0;
        return;
    }

    if (s_peri_adv_retry_count < kPeriAdvMaxRetries && s_peri_adv_timer != nullptr) {
        s_peri_adv_retry_count++;
        s_peri_adv_retry_armed = true;
        ESP_LOGW(TAG, "ADV restart retry %d/%d",
                 s_peri_adv_retry_count, kPeriAdvMaxRetries);
        (void)xTimerChangePeriod(s_peri_adv_timer, kPeriAdvRetryDelayTicks, 0);
        (void)xTimerStart(s_peri_adv_timer, 0);
        return;
    }
    s_peri_adv_retry_armed = false;
    s_peri_adv_retry_count = 0;
    ESP_LOGE(TAG, "ADV restart gave up after retries");
}

static void periScheduleAdvertiseRestart(void)
{
    if (s_peri == nullptr || !s_peri->getConfig().auto_advertise) {
        return;
    }
    if (s_peri_adv_timer == nullptr) {
        ESP_LOGE(TAG, "ADV restart timer missing");
        return;
    }
    s_peri_adv_retry_armed = false;
    s_peri_adv_retry_count = 0;
    (void)xTimerChangePeriod(s_peri_adv_timer, kPeriAdvRestartDelayTicks, 0);
    if (xTimerStart(s_peri_adv_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "ADV restart timer start failed");
    } else {
        ESP_LOGI(TAG, "ADV restart scheduled in %u ms",
                 (unsigned)(kPeriAdvRestartDelayTicks * portTICK_PERIOD_MS));
    }
}

static esp_err_t periCreateAdvTimer(void)
{
    if (s_peri_adv_timer != nullptr) {
        return ESP_OK;
    }
    s_peri_adv_timer = xTimerCreate(
        "bd_peri_adv", kPeriAdvRestartDelayTicks, pdFALSE, nullptr, periAdvTimerCb);
    if (s_peri_adv_timer == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void periDeleteAdvTimer(void)
{
    if (s_peri_adv_timer == nullptr) {
        return;
    }
    (void)xTimerStop(s_peri_adv_timer, 0);
    (void)xTimerDelete(s_peri_adv_timer, 0);
    s_peri_adv_timer = nullptr;
    s_peri_adv_retry_armed = false;
}

static esp_err_t periNotify(uint16_t attr_handle, bool subscribed,
                            const uint8_t* data, size_t len)
{
    if (!s_stack_ready || s_peri_gatts_if == ESP_GATT_IF_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_peri_conn_id == kBdInvalidConn) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!subscribed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || len == 0U || len > kBluedroidMaxPayloadBytes) {
        return ESP_ERR_INVALID_ARG;
    }
    if (attr_handle == kBdInvalidHandle) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_ble_gatts_send_indicate(
        s_peri_gatts_if, s_peri_conn_id, attr_handle,
        static_cast<uint16_t>(len), const_cast<uint8_t*>(data), false);
}


static const bluedroid_sec_cfg_t* activeSecCfg(void)
{
    if (s_peri != nullptr) {
        return &s_peri->getConfig().sec;
    }
    if (s_cent != nullptr) {
        return &s_cent->getConfig().sec;
    }
    return nullptr;
}

static esp_ble_auth_req_t toEspAuth(eBluedroidAuthMode mode)
{
    switch (mode) {
    case eBluedroidAuthMode::Bond:
        return ESP_LE_AUTH_BOND;
    case eBluedroidAuthMode::ScMitmBond:
        return ESP_LE_AUTH_REQ_SC_MITM_BOND;
    case eBluedroidAuthMode::None:
    default:
        return ESP_LE_AUTH_NO_BOND;
    }
}

static esp_ble_io_cap_t toEspIoCap(eBluedroidIoCap cap)
{
    switch (cap) {
    case eBluedroidIoCap::DisplayOnly:
        return ESP_IO_CAP_OUT;
    case eBluedroidIoCap::DisplayYesNo:
        return ESP_IO_CAP_IO;
    case eBluedroidIoCap::KeyboardOnly:
        return ESP_IO_CAP_IN;
    case eBluedroidIoCap::KeyboardDisplay:
        return ESP_IO_CAP_KBDISP;
    case eBluedroidIoCap::None:
    default:
        return ESP_IO_CAP_NONE;
    }
}

static esp_ble_sec_act_t toEspSecAct(eBluedroidAuthMode mode)
{
    if (mode == eBluedroidAuthMode::ScMitmBond) {
        return ESP_BLE_SEC_ENCRYPT_MITM;
    }
    return ESP_BLE_SEC_ENCRYPT_NO_MITM;
}

static esp_err_t applySecurityParams(const bluedroid_sec_cfg_t& sec)
{
    if (!sec.enable) {
        return ESP_OK;
    }

    esp_ble_auth_req_t auth_req = toEspAuth(sec.auth_mode);
    esp_ble_io_cap_t iocap = toEspIoCap(sec.io_cap);
    uint8_t key_size = kBdSmMaxKeySize;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = sec.static_passkey;
    if (passkey > 999999U) {
        passkey = kBluedroidDefaultStaticPasskey;
    }

    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(passkey)),
        TAG, "SM passkey");
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req)),
        TAG, "SM auth");
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap)),
        TAG, "SM iocap");
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size)),
        TAG, "SM key_size");
    {
        uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
        uint8_t oob_support = ESP_BLE_OOB_DISABLE;
        ESP_RETURN_ON_ERROR(
            esp_ble_gap_set_security_param(
                ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                &auth_option, sizeof(auth_option)),
            TAG, "SM auth_option");
        ESP_RETURN_ON_ERROR(
            esp_ble_gap_set_security_param(
                ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(oob_support)),
            TAG, "SM oob");
    }
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key)),
        TAG, "SM init_key");
    ESP_RETURN_ON_ERROR(
        esp_ble_gap_set_security_param(
            ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key)),
        TAG, "SM rsp_key");

    ESP_LOGI(TAG, "SMP enabled auth=%d iocap=%d passkey=%06" PRIu32,
             (int)sec.auth_mode, (int)sec.io_cap, passkey);
    return ESP_OK;
}

static esp_err_t requestEncryptionOnBda(const esp_bd_addr_t bda)
{
    const bluedroid_sec_cfg_t* sec = activeSecCfg();
    if (sec == nullptr || !sec->enable) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    const esp_ble_sec_act_t act = toEspSecAct(sec->auth_mode);
    const esp_err_t err = esp_ble_set_encryption(const_cast<uint8_t*>(bda), act);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_encryption: %s", esp_err_to_name(err));
    }
    return err;
}

static int bondCountNow(void)
{
    return esp_ble_get_bond_device_num();
}

static esp_err_t removeBondAddr(const bluedroid_bd_addr_t& addr)
{
    esp_bd_addr_t bda = {};
    memcpy(bda, addr.bytes, sizeof(bda));
    return esp_ble_remove_bond_device(bda);
}

static esp_err_t clearAllBonds(void)
{
    const int n = esp_ble_get_bond_device_num();
    if (n <= 0) {
        return ESP_OK;
    }
    esp_ble_bond_dev_t* list =
        static_cast<esp_ble_bond_dev_t*>(malloc(sizeof(esp_ble_bond_dev_t) * (size_t)n));
    if (list == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    int count = n;
    esp_err_t err = esp_ble_get_bond_device_list(&count, list);
    if (err != ESP_OK) {
        free(list);
        return err;
    }
    for (int i = 0; i < count; ++i) {
        (void)esp_ble_remove_bond_device(list[i].bd_addr);
    }
    free(list);
    return ESP_OK;
}

static bool handleSecGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
        (void)esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        return true;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: {
        const uint32_t passkey = param->ble_security.key_notif.passkey;
        ESP_LOGI(TAG, "passkey display: %06" PRIu32, passkey);
        if (s_passkey_cb != nullptr) {
            s_passkey_cb(passkey);
        }
        return true;
    }

    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(TAG, "NC confirm passkey=%06" PRIu32,
                 param->ble_security.key_notif.passkey);
        (void)esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        return true;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        /* Static passkey is configured; stack uses it for DisplayOnly. */
        return true;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        const bool ok = param->ble_security.auth_cmpl.success;
        const uint8_t reason = param->ble_security.auth_cmpl.fail_reason;
        s_link_encrypted = ok;
        s_link_bonded = ok;
        if (ok) {
            ESP_LOGI(TAG, "AUTH_CMPL success");
        } else {
            ESP_LOGW(TAG, "AUTH_CMPL fail reason=0x%x", reason);
        }
        if (s_auth_cb != nullptr) {
            s_auth_cb(ok, reason);
        }
        return true;
    }

    case ESP_GAP_BLE_KEY_EVT:
        return true;

    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
        return true;

    default:
        return false;
    }
}

static void periAddNextChar(void)
{
    /* Step 0 RX write, 1 TX notify, 2 status notify. */
    const bluedroid_sec_cfg_t* sec = activeSecCfg();
    const bool enc_gatt =
        (sec != nullptr && sec->enable && sec->require_enc_gatt);
    const esp_gatt_perm_t rx_perm =
        enc_gatt ? ESP_GATT_PERM_WRITE_ENC_MITM : ESP_GATT_PERM_WRITE;
    const esp_gatt_perm_t tx_perm =
        enc_gatt ? (ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM)
                 : (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);

    esp_err_t err = ESP_OK;
    if (s_peri_char_step == 0) {
        esp_gatt_char_prop_t prop =
            ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
        err = esp_ble_gatts_add_char(
            s_peri_service_h, &s_rx_uuid, rx_perm, prop,
            nullptr, nullptr);
    } else if (s_peri_char_step == 1) {
        esp_gatt_char_prop_t prop =
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        err = esp_ble_gatts_add_char(
            s_peri_service_h, &s_tx_uuid, tx_perm, prop, nullptr, nullptr);
    } else if (s_peri_char_step == 2) {
        esp_gatt_char_prop_t prop =
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        err = esp_ble_gatts_add_char(
            s_peri_service_h, &s_st_uuid, tx_perm, prop, nullptr, nullptr);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_char step=%u: %s",
                 (unsigned)s_peri_char_step, esp_err_to_name(err));
    }
}

static void periGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        s_peri_adv_ready = (param->adv_data_raw_cmpl.status == ESP_BT_STATUS_SUCCESS);
        if (!s_peri_adv_ready) {
            ESP_LOGE(TAG, "ADV raw set failed status=%d",
                     param->adv_data_raw_cmpl.status);
            s_peri_adv_start_pending = false;
        } else {
            periTryStartAdvAfterData();
        }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        s_peri_scan_rsp_ready =
            (param->scan_rsp_data_raw_cmpl.status == ESP_BT_STATUS_SUCCESS);
        if (!s_peri_scan_rsp_ready) {
            ESP_LOGE(TAG, "SCAN_RSP raw set failed status=%d",
                     param->scan_rsp_data_raw_cmpl.status);
            s_peri_adv_start_pending = false;
        } else {
            periTryStartAdvAfterData();
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "ADV start status=%d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        break;

    default:
        break;
    }
}

static void periGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t* param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTS reg status=%d", param->reg.status);
            signalInitDone();
            break;
        }
        s_peri_gatts_if = gatts_if;
        {
            const char* name = (s_peri != nullptr && s_peri->getConfig().device_name != nullptr)
                                   ? s_peri->getConfig().device_name
                                   : kBluedroidDefaultDeviceName;
            (void)esp_ble_gap_set_device_name(name);
        }
        {
            esp_gatt_srvc_id_t sid = {};
            sid.is_primary = true;
            sid.id.inst_id = 0;
            sid.id.uuid    = s_svc_uuid;
            /* Service + 3 chars (decl+value) + 2 CCCD = 9 handles; reserve 16. */
            const esp_err_t err =
                esp_ble_gatts_create_service(gatts_if, &sid, 16);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "create_service: %s", esp_err_to_name(err));
                signalInitDone();
            }
        }
        break;

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "create status=%d", param->create.status);
            signalInitDone();
            break;
        }
        s_peri_service_h = param->create.service_handle;
        s_peri_char_step = 0;
        periAddNextChar();
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "add_char status=%d", param->add_char.status);
            signalInitDone();
            break;
        }
        if (uuidEqual(param->add_char.char_uuid, s_rx_uuid)) {
            s_peri_rx_h = param->add_char.attr_handle;
            s_peri_char_step = 1;
            periAddNextChar();
        } else if (uuidEqual(param->add_char.char_uuid, s_tx_uuid)) {
            s_peri_tx_h = param->add_char.attr_handle;
            {
                esp_bt_uuid_t cccd = {};
                cccd.len = ESP_UUID_LEN_16;
                cccd.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                {
                    const bluedroid_sec_cfg_t* sec2 = activeSecCfg();
                    const bool enc2 =
                        (sec2 != nullptr && sec2->enable && sec2->require_enc_gatt);
                    const esp_gatt_perm_t cccd_perm =
                        enc2 ? (ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM)
                             : (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
                    (void)esp_ble_gatts_add_char_descr(
                        s_peri_service_h, &cccd, cccd_perm, nullptr, nullptr);
                }
            }
        } else if (uuidEqual(param->add_char.char_uuid, s_st_uuid)) {
            s_peri_st_h = param->add_char.attr_handle;
            {
                esp_bt_uuid_t cccd = {};
                cccd.len = ESP_UUID_LEN_16;
                cccd.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
                {
                    const bluedroid_sec_cfg_t* sec2 = activeSecCfg();
                    const bool enc2 =
                        (sec2 != nullptr && sec2->enable && sec2->require_enc_gatt);
                    const esp_gatt_perm_t cccd_perm =
                        enc2 ? (ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM)
                             : (ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
                    (void)esp_ble_gatts_add_char_descr(
                        s_peri_service_h, &cccd, cccd_perm, nullptr, nullptr);
                }
            }
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "add_descr status=%d", param->add_char_descr.status);
            signalInitDone();
            break;
        }
        if (s_peri_tx_cccd_h == kBdInvalidHandle) {
            s_peri_tx_cccd_h = param->add_char_descr.attr_handle;
            s_peri_char_step = 2;
            periAddNextChar();
        } else if (s_peri_st_cccd_h == kBdInvalidHandle) {
            s_peri_st_cccd_h = param->add_char_descr.attr_handle;
            (void)esp_ble_gatts_start_service(s_peri_service_h);
        }
        break;

    case ESP_GATTS_START_EVT:
        if (param->start.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "start_service status=%d", param->start.status);
            signalInitDone();
            break;
        }
        s_peri_service_started = true;
        ESP_LOGI(TAG, "GATT service started");
        signalInitDone();
        if (s_peri != nullptr && s_peri->getConfig().auto_advertise) {
            (void)periAdvertiseNow(s_peri->getConfig());
        }
        break;

    case ESP_GATTS_CONNECT_EVT: {
        portENTER_CRITICAL(&s_bd_mux);
        s_peri_conn_id = param->connect.conn_id;
        memcpy(s_peri_peer_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        s_link_encrypted = false;
        s_link_bonded = false;
        bluedroid_conn_cb_t cb = s_peri_conn_cb;
        uint16_t cid = s_peri_conn_id;
        portEXIT_CRITICAL(&s_bd_mux);
        ESP_LOGI(TAG, "peri connected conn_id=%u", (unsigned)cid);
        (void)esp_ble_gap_stop_advertising();
        (void)esp_ble_gatt_set_local_mtu(kBdPreferredMtu);
        {
            const bluedroid_sec_cfg_t* sec = activeSecCfg();
            if (sec != nullptr && sec->enable && sec->auto_encrypt_on_connect) {
                (void)requestEncryptionOnBda(s_peri_peer_bda);
            }
        }
        if (cb != nullptr) {
            cb(true, cid);
        }
        break;
    }

    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(TAG, "peri disconnect reason=0x%x",
                 param->disconnect.reason);
        portENTER_CRITICAL(&s_bd_mux);
        const uint16_t old = s_peri_conn_id;
        s_peri_conn_id = kBdInvalidConn;
        s_peri_tx_sub = false;
        s_peri_st_sub = false;
        s_link_encrypted = false;
        s_link_bonded = false;
        bluedroid_conn_cb_t cb = s_peri_conn_cb;
        portEXIT_CRITICAL(&s_bd_mux);
        if (cb != nullptr) {
            cb(false, old);
        }
        periScheduleAdvertiseRestart();
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        if (param->write.handle == s_peri_rx_h) {
            bluedroid_data_cb_t cb = s_peri_rx_cb;
            if (cb != nullptr && param->write.len > 0U && param->write.value != nullptr) {
                cb(param->write.value, param->write.len);
            }
        } else if (param->write.handle == s_peri_tx_cccd_h && param->write.len >= 2U) {
            const bool en = (param->write.value[0] & 0x01U) != 0U;
            portENTER_CRITICAL(&s_bd_mux);
            s_peri_tx_sub = en;
            portEXIT_CRITICAL(&s_bd_mux);
            ESP_LOGI(TAG, "TX CCCD notify=%d", (int)en);
        } else if (param->write.handle == s_peri_st_cccd_h && param->write.len >= 2U) {
            const bool en = (param->write.value[0] & 0x01U) != 0U;
            portENTER_CRITICAL(&s_bd_mux);
            s_peri_st_sub = en;
            portEXIT_CRITICAL(&s_bd_mux);
            ESP_LOGI(TAG, "status CCCD notify=%d", (int)en);
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(
                gatts_if, param->write.conn_id, param->write.trans_id,
                ESP_GATT_OK, nullptr);
        }
        break;
    }

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "ATT MTU conn=%u mtu=%u",
                 (unsigned)param->mtu.conn_id, (unsigned)param->mtu.mtu);
        break;

    default:
        break;
    }
}

/* ── Central GAP / GATTC ─────────────────────────────────────────────────── */

static bool advParseMatch(uint8_t* adv, uint8_t adv_len, const char* name_filter)
{
    bool uuid_ok = (name_filter != nullptr && name_filter[0] != '\0') ? false : true;
    bool name_ok = (name_filter == nullptr || name_filter[0] == '\0');
    bool saw_uuid = false;

    uint8_t* p = adv;
    uint8_t* end = adv + adv_len;
    while (p + 1 < end) {
        const uint8_t field_len = p[0];
        if (field_len == 0U || p + 1 + field_len > end) {
            break;
        }
        const uint8_t type = p[1];
        const uint8_t* val = &p[2];
        const uint8_t vlen = static_cast<uint8_t>(field_len - 1U);

        if ((type == ESP_BLE_AD_TYPE_NAME_CMPL || type == ESP_BLE_AD_TYPE_NAME_SHORT) &&
            name_filter != nullptr && name_filter[0] != '\0') {
            const size_t flen = strlen(name_filter);
            name_ok = (flen == vlen) && (memcmp(val, name_filter, flen) == 0);
        }
        if (type == ESP_BLE_AD_TYPE_128SRV_CMPL || type == ESP_BLE_AD_TYPE_128SRV_PART) {
            saw_uuid = true;
            uuid_ok = false;
            for (uint8_t i = 0; i + 16U <= vlen; i = static_cast<uint8_t>(i + 16U)) {
                if (memcmp(&val[i], s_svc_uuid.uuid.uuid128, 16) == 0) {
                    uuid_ok = true;
                    break;
                }
            }
        }
        p = p + 1 + field_len;
    }

    if (name_filter != nullptr && name_filter[0] != '\0') {
        /* Name filter: accept name match, or UUID match if name absent. */
        return name_ok || (saw_uuid && uuid_ok);
    }
    return uuid_ok && saw_uuid;
}

static void centGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
                s_cent_scanning = false;
                ESP_LOGI(TAG, "scan complete");
            }
            break;
        }
        if (s_cent_connecting || s_cent_conn_id != kBdInvalidConn) {
            break;
        }
        const char* filter =
            (s_cent != nullptr) ? s_cent->getConfig().peer_name_filter : nullptr;
        if (!advParseMatch(param->scan_rst.ble_adv, param->scan_rst.adv_data_len, filter) &&
            !advParseMatch(param->scan_rst.ble_adv + param->scan_rst.adv_data_len,
                           param->scan_rst.scan_rsp_len, filter)) {
            /* Also try full buffer length (adv + optional rsp concatenated). */
            const uint8_t total =
                static_cast<uint8_t>(param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len);
            if (!advParseMatch(param->scan_rst.ble_adv, total, filter)) {
                break;
            }
        }

        s_cent_connecting = true;
        memcpy(s_cent_peer_bda, param->scan_rst.bda, sizeof(esp_bd_addr_t));
        s_cent_peer_addr_type = param->scan_rst.ble_addr_type;
        (void)esp_ble_gap_stop_scanning();
        s_cent_scanning = false;
        ESP_LOGI(TAG, "peer found — connect");
        const esp_err_t err = esp_ble_gattc_open(
            s_cent_gattc_if, s_cent_peer_bda, s_cent_peer_addr_type, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gattc open: %s", esp_err_to_name(err));
            s_cent_connecting = false;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        s_cent_scanning = false;
        break;

    default:
        break;
    }
}

static void centRegisterNotify(uint16_t char_handle, uint16_t cccd_handle, bool enable)
{
    if (s_cent_gattc_if == ESP_GATT_IF_NONE || s_cent_conn_id == kBdInvalidConn) {
        return;
    }
    if (enable) {
        (void)esp_ble_gattc_register_for_notify(
            s_cent_gattc_if, s_cent_peer_bda, char_handle);
    } else {
        (void)esp_ble_gattc_unregister_for_notify(
            s_cent_gattc_if, s_cent_peer_bda, char_handle);
    }
    if (cccd_handle != kBdInvalidHandle) {
        uint8_t val[2] = {static_cast<uint8_t>(enable ? 0x01 : 0x00), 0x00};
        (void)esp_ble_gattc_write_char_descr(
            s_cent_gattc_if, s_cent_conn_id, cccd_handle, sizeof(val), val,
            ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    }
}

static void centGattcEvent(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t* param)
{
    switch (event) {  // NOLINT(clang::switch)
    case ESP_GATTC_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "GATTC reg status=%d", param->reg.status);
            signalInitDone();
            break;
        }
        s_cent_gattc_if = gattc_if;
        ESP_LOGI(TAG, "GATTC registered");
        signalInitDone();
        break;

    case ESP_GATTC_CONNECT_EVT: {
        portENTER_CRITICAL(&s_bd_mux);
        s_cent_conn_id = param->connect.conn_id;
        s_cent_connecting = false;
        s_link_encrypted = false;
        s_link_bonded = false;
        bluedroid_conn_cb_t cb = s_cent_conn_cb;
        uint16_t cid = s_cent_conn_id;
        portEXIT_CRITICAL(&s_bd_mux);
        memcpy(s_cent_peer_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        {
            const bluedroid_sec_cfg_t* sec = activeSecCfg();
            if (sec != nullptr && sec->enable && sec->auto_encrypt_on_connect) {
                (void)requestEncryptionOnBda(s_cent_peer_bda);
            }
        }
        ESP_LOGI(TAG, "cent connected conn_id=%u", (unsigned)cid);
        if (cb != nullptr) {
            cb(true, cid);
        }
        (void)esp_ble_gattc_send_mtu_req(gattc_if, cid);
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
        ESP_LOGI(TAG, "cent disconnect reason=0x%x", param->disconnect.reason);
        portENTER_CRITICAL(&s_bd_mux);
        const uint16_t old = s_cent_conn_id;
        s_cent_conn_id = kBdInvalidConn;
        s_cent_ready = false;
        s_cent_connecting = false;
        s_cent_rx_h = s_cent_tx_h = s_cent_st_h = kBdInvalidHandle;
        s_cent_tx_cccd = s_cent_st_cccd = kBdInvalidHandle;
        bluedroid_conn_cb_t dcb = s_cent_conn_cb;
        portEXIT_CRITICAL(&s_bd_mux);
        if (dcb != nullptr) {
            dcb(false, old);
        }
        break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG, "cent MTU=%u", (unsigned)param->cfg_mtu.mtu);
        (void)esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &s_svc_uuid);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "service search status=%d", param->search_cmpl.status);
            break;
        }
        constexpr uint16_t kMaxChars = 16U;
        constexpr uint16_t kMaxDescr = 8U;
        esp_gattc_char_elem_t chars[kMaxChars];
        uint16_t count = kMaxChars;
        esp_gatt_status_t st = esp_ble_gattc_get_attr_count(
            gattc_if, param->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
            0, 0xFFFF, 0, &count);
        if (st != ESP_GATT_OK || count == 0U) {
            ESP_LOGW(TAG, "no characteristics count=%u st=%d", (unsigned)count, st);
            break;
        }
        if (count > kMaxChars) {
            count = kMaxChars;
        }
        st = esp_ble_gattc_get_all_char(
            gattc_if, param->search_cmpl.conn_id, 0, 0xFFFF, chars, &count, 0);
        if (st == ESP_GATT_OK) {
            for (uint16_t i = 0; i < count; ++i) {
                if (uuidEqual(chars[i].uuid, s_rx_uuid)) {
                    s_cent_rx_h = chars[i].char_handle;
                } else if (uuidEqual(chars[i].uuid, s_tx_uuid)) {
                    s_cent_tx_h = chars[i].char_handle;
                } else if (uuidEqual(chars[i].uuid, s_st_uuid)) {
                    s_cent_st_h = chars[i].char_handle;
                }
            }
        }

        auto resolve_cccd = [&](uint16_t char_h, uint16_t* out_cccd) {
            esp_gattc_descr_elem_t descr[kMaxDescr];
            uint16_t dcount = kMaxDescr;
            if (esp_ble_gattc_get_attr_count(
                    gattc_if, param->search_cmpl.conn_id, ESP_GATT_DB_DESCRIPTOR,
                    char_h, char_h, char_h, &dcount) != ESP_GATT_OK ||
                dcount == 0U) {
                return;
            }
            if (dcount > kMaxDescr) {
                dcount = kMaxDescr;
            }
            if (esp_ble_gattc_get_all_descr(
                    gattc_if, param->search_cmpl.conn_id, char_h, descr, &dcount,
                    0) != ESP_GATT_OK) {
                return;
            }
            for (uint16_t i = 0; i < dcount; ++i) {
                if (descr[i].uuid.len == ESP_UUID_LEN_16 &&
                    descr[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                    *out_cccd = descr[i].handle;
                    return;
                }
            }
        };
        if (s_cent_tx_h != kBdInvalidHandle) {
            resolve_cccd(s_cent_tx_h, &s_cent_tx_cccd);
        }
        if (s_cent_st_h != kBdInvalidHandle) {
            resolve_cccd(s_cent_st_h, &s_cent_st_cccd);
        }

        if (s_cent_rx_h != kBdInvalidHandle) {
            s_cent_ready = true;
            ESP_LOGI(TAG, "cent ready rx=%u tx=%u st=%u",
                     (unsigned)s_cent_rx_h, (unsigned)s_cent_tx_h,
                     (unsigned)s_cent_st_h);
            if (s_cent_tx_h != kBdInvalidHandle) {
                centRegisterNotify(s_cent_tx_h, s_cent_tx_cccd, true);
            }
            if (s_cent_st_h != kBdInvalidHandle) {
                centRegisterNotify(s_cent_st_h, s_cent_st_cccd, true);
            }
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.handle == s_cent_tx_h) {
            if (s_cent_tx_cb != nullptr) {
                s_cent_tx_cb(param->notify.value, param->notify.value_len);
            }
        } else if (param->notify.handle == s_cent_st_h) {
            if (s_cent_status_cb != nullptr) {
                s_cent_status_cb(param->notify.value, param->notify.value_len);
            }
        }
        break;

    default:
        break;
    }
}

static void gapDispatch(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    if (handleSecGapEvent(event, param)) {
        return;
    }
    if (s_host_role == eBdHostPeripheral) {
        periGapEvent(event, param);
    } else if (s_host_role == eBdHostCentral) {
        centGapEvent(event, param);
    }
}

/* ── BluedroidPeripheral ─────────────────────────────────────────────────── */

BluedroidPeripheral::BluedroidPeripheral(const bluedroid_cfg_t& config)
    : m_config(config)
{
}

BluedroidPeripheral::~BluedroidPeripheral()
{
    if (m_inited) {
        (void)deinit();
    }
}

esp_err_t BluedroidPeripheral::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "peripheral already inited");
        return ESP_OK;
    }
    if (s_host_role != eBdHostIdle) {
        ESP_LOGE(TAG, "BLE host busy (role=%d)", (int)s_host_role);
        return ESP_ERR_INVALID_STATE;
    }

    loadUuid(&s_svc_uuid, m_config.service_uuid);
    loadUuid(&s_rx_uuid, m_config.rx_uuid);
    loadUuid(&s_tx_uuid, m_config.tx_uuid);
    loadUuid(&s_st_uuid, m_config.status_uuid);

    s_peri_gatts_if = ESP_GATT_IF_NONE;
    s_peri_conn_id = kBdInvalidConn;
    s_peri_service_h = s_peri_rx_h = s_peri_tx_h = s_peri_st_h = kBdInvalidHandle;
    s_peri_tx_cccd_h = s_peri_st_cccd_h = kBdInvalidHandle;
    s_peri_tx_sub = s_peri_st_sub = false;
    s_peri_service_started = false;
    s_peri_char_step = 0;

    ESP_RETURN_ON_ERROR(periCreateAdvTimer(), TAG, "periCreateAdvTimer");
    ESP_RETURN_ON_ERROR(startControllerAndBluedroid(), TAG, "start stack");

    if (s_init_sem == nullptr) {
        s_init_sem = xSemaphoreCreateBinary();
        if (s_init_sem == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }
    (void)xSemaphoreTake(s_init_sem, 0);

    s_host_role = eBdHostPeripheral;
    s_peri = this;
    s_peri_conn_cb = m_conn_cb;
    s_peri_rx_cb = m_rx_cb;
    s_auth_cb = m_auth_cb;
    s_passkey_cb = m_passkey_cb;
    s_link_encrypted = false;
    s_link_bonded = false;

    ESP_RETURN_ON_ERROR(esp_ble_gap_register_callback(gapDispatch), TAG, "gap_cb");
    ESP_RETURN_ON_ERROR(applySecurityParams(m_config.sec), TAG, "SMP params");
    ESP_RETURN_ON_ERROR(esp_ble_gatts_register_callback(periGattsEvent), TAG, "gatts_cb");
    ESP_RETURN_ON_ERROR(esp_ble_gatts_app_register(s_peri_app_id), TAG, "gatts_app_reg");

    if (xSemaphoreTake(s_init_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "GATTS register timeout");
        s_host_role = eBdHostIdle;
        s_peri = nullptr;
        return ESP_ERR_TIMEOUT;
    }
    if (!s_peri_service_started) {
        ESP_LOGE(TAG, "GATT service did not start");
        s_host_role = eBdHostIdle;
        s_peri = nullptr;
        return ESP_FAIL;
    }

    m_inited = true;
    ESP_LOGI(TAG, "BluedroidPeripheral ready");
    return ESP_OK;
}

esp_err_t BluedroidPeripheral::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }
    (void)stopAdvertising();
    periDeleteAdvTimer();

    if (s_peri_gatts_if != ESP_GATT_IF_NONE) {
        (void)esp_ble_gatts_app_unregister(s_peri_gatts_if);
        s_peri_gatts_if = ESP_GATT_IF_NONE;
    }

    s_peri = nullptr;
    s_peri_conn_cb = nullptr;
    s_peri_rx_cb = nullptr;
    if (s_cent == nullptr) {
        s_auth_cb = nullptr;
        s_passkey_cb = nullptr;
    }
    s_host_role = eBdHostIdle;
    m_inited = false;

    if (s_cent == nullptr) {
        (void)stopControllerAndBluedroid();
    }
    return ESP_OK;
}

esp_err_t BluedroidPeripheral::startAdvertising()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return periRestartAdvertising(m_config);
}

esp_err_t BluedroidPeripheral::stopAdvertising()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_ble_gap_stop_advertising();
}

esp_err_t BluedroidPeripheral::notifyTx(const uint8_t* data, size_t len)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return periNotify(s_peri_tx_h, s_peri_tx_sub, data, len);
}

esp_err_t BluedroidPeripheral::notifyStatus(const uint8_t* data, size_t len)
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    return periNotify(s_peri_st_h, s_peri_st_sub, data, len);
}

esp_err_t BluedroidPeripheral::notifyStatus(const char* utf8)
{
    if (utf8 == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return notifyStatus(reinterpret_cast<const uint8_t*>(utf8), strlen(utf8));
}

void BluedroidPeripheral::setConnCallback(bluedroid_conn_cb_t cb)
{
    m_conn_cb = cb;
    s_peri_conn_cb = cb;
}

void BluedroidPeripheral::setRxCallback(bluedroid_data_cb_t cb)
{
    m_rx_cb = cb;
    s_peri_rx_cb = cb;
}

void BluedroidPeripheral::setAuthCallback(bluedroid_auth_cb_t cb)
{
    m_auth_cb = cb;
    s_auth_cb = cb;
}

void BluedroidPeripheral::setPasskeyDisplayCallback(bluedroid_passkey_cb_t cb)
{
    m_passkey_cb = cb;
    s_passkey_cb = cb;
}

esp_err_t BluedroidPeripheral::requestEncryption()
{
    if (!m_inited || s_peri_conn_id == kBdInvalidConn) {
        return ESP_ERR_INVALID_STATE;
    }
    return requestEncryptionOnBda(s_peri_peer_bda);
}

int BluedroidPeripheral::getBondCount() const
{
    return bondCountNow();
}

esp_err_t BluedroidPeripheral::removeBond(const bluedroid_bd_addr_t& addr)
{
    return removeBondAddr(addr);
}

esp_err_t BluedroidPeripheral::clearBonds()
{
    return clearAllBonds();
}

bool BluedroidPeripheral::isEncrypted() const
{
    return m_inited && s_link_encrypted;
}

bool BluedroidPeripheral::isBonded() const
{
    return m_inited && s_link_bonded;
}

bool BluedroidPeripheral::isConnected() const
{
    return m_inited && s_peri_conn_id != kBdInvalidConn;
}

bool BluedroidPeripheral::isTxSubscribed() const
{
    return m_inited && s_peri_tx_sub;
}

bool BluedroidPeripheral::isStatusSubscribed() const
{
    return m_inited && s_peri_st_sub;
}

bool BluedroidPeripheral::isInited() const
{
    return m_inited;
}

const bluedroid_cfg_t& BluedroidPeripheral::getConfig() const
{
    return m_config;
}

/* ── BluedroidCentral ────────────────────────────────────────────────────── */

BluedroidCentral::BluedroidCentral(const bluedroid_cfg_t& config)
    : m_config(config)
{
}

BluedroidCentral::~BluedroidCentral()
{
    if (m_inited) {
        (void)deinit();
    }
}

esp_err_t BluedroidCentral::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "central already inited");
        return ESP_OK;
    }
    if (s_host_role != eBdHostIdle) {
        ESP_LOGE(TAG, "BLE host busy (role=%d)", (int)s_host_role);
        return ESP_ERR_INVALID_STATE;
    }

    loadUuid(&s_svc_uuid, m_config.service_uuid);
    loadUuid(&s_rx_uuid, m_config.rx_uuid);
    loadUuid(&s_tx_uuid, m_config.tx_uuid);
    loadUuid(&s_st_uuid, m_config.status_uuid);

    s_cent_gattc_if = ESP_GATT_IF_NONE;
    s_cent_conn_id = kBdInvalidConn;
    s_cent_scanning = s_cent_connecting = s_cent_ready = false;
    s_cent_rx_h = s_cent_tx_h = s_cent_st_h = kBdInvalidHandle;
    s_cent_tx_cccd = s_cent_st_cccd = kBdInvalidHandle;

    ESP_RETURN_ON_ERROR(startControllerAndBluedroid(), TAG, "start stack");

    if (s_init_sem == nullptr) {
        s_init_sem = xSemaphoreCreateBinary();
        if (s_init_sem == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }
    (void)xSemaphoreTake(s_init_sem, 0);

    s_host_role = eBdHostCentral;
    s_cent = this;
    s_cent_conn_cb = m_conn_cb;
    s_cent_tx_cb = m_tx_cb;
    s_cent_status_cb = m_status_cb;
    s_auth_cb = m_auth_cb;
    s_passkey_cb = m_passkey_cb;
    s_link_encrypted = false;
    s_link_bonded = false;

    ESP_RETURN_ON_ERROR(esp_ble_gap_register_callback(gapDispatch), TAG, "gap_cb");
    ESP_RETURN_ON_ERROR(applySecurityParams(m_config.sec), TAG, "SMP params");
    ESP_RETURN_ON_ERROR(esp_ble_gattc_register_callback(centGattcEvent), TAG, "gattc_cb");
    ESP_RETURN_ON_ERROR(esp_ble_gattc_app_register(s_cent_app_id), TAG, "gattc_app_reg");

    if (xSemaphoreTake(s_init_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "GATTC register timeout");
        s_host_role = eBdHostIdle;
        s_cent = nullptr;
        return ESP_ERR_TIMEOUT;
    }

    m_inited = true;
    ESP_LOGI(TAG, "BluedroidCentral ready");
    return ESP_OK;
}

esp_err_t BluedroidCentral::deinit()
{
    if (!m_inited) {
        return ESP_OK;
    }
    (void)stopScan();
    (void)disconnect();

    if (s_cent_gattc_if != ESP_GATT_IF_NONE) {
        (void)esp_ble_gattc_app_unregister(s_cent_gattc_if);
        s_cent_gattc_if = ESP_GATT_IF_NONE;
    }

    s_cent = nullptr;
    s_cent_conn_cb = nullptr;
    s_cent_tx_cb = nullptr;
    s_cent_status_cb = nullptr;
    s_host_role = eBdHostIdle;
    m_inited = false;

    if (s_peri == nullptr) {
        (void)stopControllerAndBluedroid();
    }
    return ESP_OK;
}

esp_err_t BluedroidCentral::startScan()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cent_conn_id != kBdInvalidConn || s_cent_connecting) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_ble_scan_params_t params = {};
    params.scan_type          = BLE_SCAN_TYPE_ACTIVE;
    params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
    params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
    params.scan_interval      = 0x50;
    params.scan_window        = 0x30;
    params.scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE;
    ESP_RETURN_ON_ERROR(esp_ble_gap_set_scan_params(&params), TAG, "scan_params");

    uint32_t duration_s = 0;
    if (m_config.scan_ms > 0U) {
        duration_s = (m_config.scan_ms + 999U) / 1000U;
        if (duration_s == 0U) {
            duration_s = 1U;
        }
    }
    const esp_err_t err = esp_ble_gap_start_scanning(duration_s);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start_scanning: %s", esp_err_to_name(err));
        return err;
    }
    s_cent_scanning = true;
    ESP_LOGI(TAG, "scanning (%u s, 0=forever)", (unsigned)duration_s);
    return ESP_OK;
}

esp_err_t BluedroidCentral::stopScan()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cent_scanning = false;
    return esp_ble_gap_stop_scanning();
}

esp_err_t BluedroidCentral::disconnect()
{
    if (!m_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_cent_conn_id == kBdInvalidConn || s_cent_gattc_if == ESP_GATT_IF_NONE) {
        return ESP_OK;
    }
    return esp_ble_gattc_close(s_cent_gattc_if, s_cent_conn_id);
}

esp_err_t BluedroidCentral::writeRx(const uint8_t* data, size_t len)
{
    if (!m_inited || !s_cent_ready || s_cent_conn_id == kBdInvalidConn) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == nullptr || len == 0U || len > kBluedroidMaxPayloadBytes) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_cent_rx_h == kBdInvalidHandle) {
        return ESP_ERR_INVALID_STATE;
    }
    const bluedroid_sec_cfg_t* sec = activeSecCfg();
    const esp_gatt_auth_req_t auth =
        (sec != nullptr && sec->enable && sec->require_enc_gatt)
            ? ESP_GATT_AUTH_REQ_MITM
            : ESP_GATT_AUTH_REQ_NONE;
    return esp_ble_gattc_write_char(
        s_cent_gattc_if, s_cent_conn_id, s_cent_rx_h,
        static_cast<uint16_t>(len), const_cast<uint8_t*>(data),
        ESP_GATT_WRITE_TYPE_RSP, auth);
}

esp_err_t BluedroidCentral::writeRx(const char* utf8)
{
    if (utf8 == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return writeRx(reinterpret_cast<const uint8_t*>(utf8), strlen(utf8));
}

esp_err_t BluedroidCentral::subscribeTx(bool enable)
{
    if (!m_inited || s_cent_tx_h == kBdInvalidHandle) {
        return ESP_ERR_INVALID_STATE;
    }
    centRegisterNotify(s_cent_tx_h, s_cent_tx_cccd, enable);
    return ESP_OK;
}

esp_err_t BluedroidCentral::subscribeStatus(bool enable)
{
    if (!m_inited || s_cent_st_h == kBdInvalidHandle) {
        return ESP_ERR_INVALID_STATE;
    }
    centRegisterNotify(s_cent_st_h, s_cent_st_cccd, enable);
    return ESP_OK;
}

void BluedroidCentral::setConnCallback(bluedroid_conn_cb_t cb)
{
    m_conn_cb = cb;
    s_cent_conn_cb = cb;
}

void BluedroidCentral::setTxNotifyCallback(bluedroid_data_cb_t cb)
{
    m_tx_cb = cb;
    s_cent_tx_cb = cb;
}

void BluedroidCentral::setStatusNotifyCallback(bluedroid_data_cb_t cb)
{
    m_status_cb = cb;
    s_cent_status_cb = cb;
}

void BluedroidCentral::setAuthCallback(bluedroid_auth_cb_t cb)
{
    m_auth_cb = cb;
    s_auth_cb = cb;
}

void BluedroidCentral::setPasskeyDisplayCallback(bluedroid_passkey_cb_t cb)
{
    m_passkey_cb = cb;
    s_passkey_cb = cb;
}

esp_err_t BluedroidCentral::requestEncryption()
{
    if (!m_inited || s_cent_conn_id == kBdInvalidConn) {
        return ESP_ERR_INVALID_STATE;
    }
    return requestEncryptionOnBda(s_cent_peer_bda);
}

int BluedroidCentral::getBondCount() const
{
    return bondCountNow();
}

esp_err_t BluedroidCentral::removeBond(const bluedroid_bd_addr_t& addr)
{
    return removeBondAddr(addr);
}

esp_err_t BluedroidCentral::clearBonds()
{
    return clearAllBonds();
}

bool BluedroidCentral::isEncrypted() const
{
    return m_inited && s_link_encrypted;
}

bool BluedroidCentral::isBonded() const
{
    return m_inited && s_link_bonded;
}

bool BluedroidCentral::isConnected() const
{
    return m_inited && s_cent_conn_id != kBdInvalidConn;
}

bool BluedroidCentral::isInited() const
{
    return m_inited;
}

bool BluedroidCentral::isReady() const
{
    return m_inited && s_cent_ready;
}

const bluedroid_cfg_t& BluedroidCentral::getConfig() const
{
    return m_config;
}

#endif /* CONFIG_BT_BLUEDROID_ENABLED */
