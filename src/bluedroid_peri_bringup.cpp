/**
 * @file        bluedroid_peri_bringup.cpp
 * @brief       HIL: BluedroidPeripheral advertise + status heartbeat
 * @details
 *   Built only by env esp32-s3-bluedroid-peri (BLUEDROID_PERI_BRINGUP=1).
 *   Connect from a phone or laptop, subscribe to status, write to RX.
 */

#include "bluedroid_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static const char* TAG = "BD_PERI_HIL";

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
    bluedroid_cfg_t cfg = {};
    BluedroidPeripheral peri(cfg);
    peri.setConnCallback(onConn);
    peri.setRxCallback(onRx);

    if (peri.init() != ESP_OK) {
        ESP_LOGE(TAG, "BluedroidPeripheral init failed");
        return;
    }

    ESP_LOGI(TAG, "Advertising as \"%s\" (Bluedroid)", cfg.device_name);

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
