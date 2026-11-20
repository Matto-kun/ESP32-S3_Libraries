/**
 * @file        bluedroid_sec_peri_bringup.cpp
 * @brief       HIL: BluedroidPeripheral with SC+MITM+bond (passkey 123456)
 * @details
 *   Env esp32-s3-bluedroid-sec-peri. ADV name ESP32-S3-BD-SEC.
 *   Pair from phone with passkey 123456 (DisplayOnly).
 */

#include "bluedroid_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>

static const char* TAG = "BD_SEC_HIL";

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
        ESP_LOGI(TAG, "AUTH_CMPL success");
    } else {
        ESP_LOGW(TAG, "AUTH_CMPL fail reason=0x%02x", fail_reason);
    }
}

static void onPasskey(uint32_t passkey)
{
    ESP_LOGI(TAG, "Enter passkey %06" PRIu32 " on the phone", passkey);
}

extern "C" void app_main(void)
{
    bluedroid_cfg_t cfg = {};
    cfg.device_name = "ESP32-S3-BD-SEC";
    cfg.sec.enable = true;
    cfg.sec.auth_mode = eBluedroidAuthMode::ScMitmBond;
    cfg.sec.io_cap = eBluedroidIoCap::DisplayOnly;
    cfg.sec.static_passkey = kBluedroidDefaultStaticPasskey;
    cfg.sec.auto_encrypt_on_connect = true;
    cfg.sec.require_enc_gatt = true;

    BluedroidPeripheral peri(cfg);
    peri.setConnCallback(onConn);
    peri.setRxCallback(onRx);
    peri.setAuthCallback(onAuth);
    peri.setPasskeyDisplayCallback(onPasskey);

    if (peri.init() != ESP_OK) {
        ESP_LOGE(TAG, "BluedroidPeripheral init failed");
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
