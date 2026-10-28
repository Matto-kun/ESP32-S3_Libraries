/**
 * @file        ble_central_example.cpp
 * @brief       Scan for ESP32-S3-BLE, connect, subscribe, write RX
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   GATT client bring-up. Peer must advertise the library default service
 *   UUID (or name filter). Requires NimBLE central + observer roles.
 */

#include "ble_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char* TAG = "BLE_CENT_EX";

static constexpr uint32_t kRescanMs = 5000U;
static constexpr uint32_t kWritePeriodMs = 3000U;

static void onConn(bool connected, uint16_t handle)
{
    ESP_LOGI(TAG, "conn=%d handle=%u", (int)connected, (unsigned)handle);
}

static void onStatus(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "status notify %u bytes", (unsigned)len);
    if (data != nullptr && len > 0U) {
        char buf[64];
        const size_t n = (len < sizeof(buf) - 1U) ? len : (sizeof(buf) - 1U);
        memcpy(buf, data, n);
        buf[n] = '\0';
        ESP_LOGI(TAG, "status=\"%s\"", buf);
    }
}

static void onTx(const uint8_t* data, size_t len)
{
    ESP_LOGI(TAG, "TX notify %u bytes", (unsigned)len);
}

extern "C" void app_main(void)
{
    ble_cfg_t cfg = {};
    cfg.peer_name_filter = kBleDefaultDeviceName;
    cfg.scan_ms = 15000U;

    BleCentral cent(cfg);
    cent.setConnCallback(onConn);
    cent.setStatusNotifyCallback(onStatus);
    cent.setTxNotifyCallback(onTx);

    if (cent.init() != ESP_OK) {
        ESP_LOGE(TAG, "BleCentral init failed");
        return;
    }

    uint32_t writes = 0;
    while (true) {
        if (!cent.isConnected()) {
            ESP_LOGI(TAG, "start scan");
            (void)cent.startScan();
            vTaskDelay(pdMS_TO_TICKS(kRescanMs));
            continue;
        }

        if (cent.isReady()) {
            char msg[32];
            snprintf(msg, sizeof(msg), "ping %lu", (unsigned long)writes);
            const esp_err_t err = cent.writeRx(msg);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "wrote \"%s\"", msg);
                ++writes;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(kWritePeriodMs));
    }
}
