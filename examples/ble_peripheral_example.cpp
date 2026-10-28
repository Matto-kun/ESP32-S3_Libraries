/**
 * @file        ble_peripheral_example.cpp
 * @brief       Advertise as ESP32-S3-BLE and send status heartbeats
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   GATT server bring-up with library default UUIDs. Connect from a phone
 *   or laptop, subscribe to status, and write to RX.
 *   Requires CONFIG_BT_ENABLED and CONFIG_BT_NIMBLE_ENABLED in the app.
 */

#include "ble_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static const char* TAG = "BLE_PERI_EX";

static constexpr uint32_t kHeartbeatMs = 2000U;

static void onConn(bool connected, uint16_t handle)
{
    ESP_LOGI(TAG, "conn=%d handle=%u", (int)connected, (unsigned)handle);
}

static void onRx(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "RX %u bytes", (unsigned)len);
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

extern "C" void app_main(void)
{
    ble_cfg_t cfg = {};
    BlePeripheral peri(cfg);
    peri.setConnCallback(onConn);
    peri.setRxCallback(onRx);

    if (peri.init() != ESP_OK) {
        ESP_LOGE(TAG, "BlePeripheral init failed");
        return;
    }

    ESP_LOGI(TAG, "Advertising as \"%s\"", cfg.device_name);

    uint32_t beat = 0;
    while (true) {
        if (peri.isConnected()) {
            char msg[48];
            snprintf(msg, sizeof(msg), "heartbeat %lu", (unsigned long)beat);
            (void)peri.notifyStatus(msg);
            ++beat;
        }
        vTaskDelay(pdMS_TO_TICKS(kHeartbeatMs));
    }
}
