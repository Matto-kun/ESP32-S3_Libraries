/**
 * @file        ble_sec_peri_bringup.cpp
 * @brief       HIL: BlePeripheral (NimBLE) with SC+MITM+bond (passkey 123456)
 * @details
 *   Env esp32-s3-ble-sec-peri. ADV name NB-SEC (short for 31-byte ADV+UUID).
 *   Pair from phone with passkey 123456 (DisplayOnly).
 */

#include "ble_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>

static const char* TAG = "NB_SEC_HIL";

static constexpr uint32_t kHeartbeatMs = 2000U;

static void onConn(bool connected, uint16_t handle)
{
    ESP_LOGI(TAG, "conn=%d handle=%u", (int)connected, (unsigned)handle);
}

static void onRx(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "RX %u bytes (encrypted GATT path)", (unsigned)len);
    if (data == nullptr || len == 0U) {
        return;
    }
    char printable[64];
    const size_t n = (len < sizeof(printable) - 1U) ? len : (sizeof(printable) - 1U);
    for (size_t i = 0; i < n; ++i) {
        const char c = static_cast<char>(data[i]);
        printable[i] = (c >= 32 && c < 127) ? c : '.';
    }
    printable[n] = '\0';
    ESP_LOGI(TAG, "RX text=\"%s\"", printable);
}

static void onAuth(bool ok, uint8_t fail_reason)
{
    if (ok) {
        ESP_LOGI(TAG, "ENC_CHANGE success");
    } else {
        ESP_LOGW(TAG, "ENC_CHANGE fail status=%u", (unsigned)fail_reason);
    }
}

static void onPasskey(uint32_t passkey)
{
    ESP_LOGI(TAG, "Enter passkey %06" PRIu32 " on the phone", passkey);
}

extern "C" void app_main(void)
{
    ble_cfg_t cfg = {};
    /* Keep name <= 8 chars so Flags+name+128-bit UUID fit in 31-byte ADV. */
    cfg.device_name = "NB-SEC";
    cfg.sec.enable = true;
    cfg.sec.auth_mode = eBleAuthMode::ScMitmBond;
    cfg.sec.io_cap = eBleIoCap::DisplayOnly;
    cfg.sec.static_passkey = kBleDefaultStaticPasskey;
    cfg.sec.auto_encrypt_on_connect = true;
    cfg.sec.require_enc_gatt = true;

    BlePeripheral peri(cfg);
    peri.setConnCallback(onConn);
    peri.setRxCallback(onRx);
    peri.setAuthCallback(onAuth);
    peri.setPasskeyDisplayCallback(onPasskey);

    if (peri.init() != ESP_OK) {
        ESP_LOGE(TAG, "BlePeripheral init failed");
        return;
    }

    ESP_LOGI(TAG, "Advertising as \"%s\" bonds=%d",
             cfg.device_name, peri.getBondCount());

    uint32_t beat = 0;
    while (true) {
        if (peri.isConnected() && peri.isEncrypted()) {
            char msg[48];
            snprintf(msg, sizeof(msg), "sec heartbeat %lu", (unsigned long)beat);
            (void)peri.notifyStatus(msg);
            ++beat;
        }
        vTaskDelay(pdMS_TO_TICKS(kHeartbeatMs));
    }
}
