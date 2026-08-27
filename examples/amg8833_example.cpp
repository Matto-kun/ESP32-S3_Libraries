/**
 * @file        amg8833_example.cpp
 * @brief       UART tester for AMG8833 Grid-EYE over shared I2C bus
 * @author      Mat RB
 * @date        2026-08-26
 *
 * @details
 *   Pins from proto_esp32_s3 pinout (example-local only):
 *   SDA = GPIO 9, SCL = GPIO 11 (same bus as HMC5883L @ 0x1E).
 *   This firmware addresses only AMG8833 (0x69, fallback 0x68).
 */

#include "i2c_api.hpp"
#include "sensors/amg8833_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static const char* TAG = "AMG_EX";

static constexpr int8_t kPinSda = 9;
static constexpr int8_t kPinScl = 11;
static constexpr uint32_t kLoopMs = 500U;

static void logFrame(const amg8833_frame_t& frame, float therm_c)
{
    float t_min = frame.pixels[0];
    float t_max = frame.pixels[0];
    for (uint8_t i = 1U; i < amg8833_regs::kPixelCount; i++) {
        if (frame.pixels[i] < t_min) {
            t_min = frame.pixels[i];
        }
        if (frame.pixels[i] > t_max) {
            t_max = frame.pixels[i];
        }
    }

    ESP_LOGI(TAG, "thermistor=%.2f C | pixel min=%.1f max=%.1f C",
             (double)therm_c, (double)t_min, (double)t_max);

    char line[96];
    for (uint8_t row = 0U; row < 8U; row++) {
        int pos = 0;
        for (uint8_t col = 0U; col < 8U; col++) {
            const float t = frame.pixels[static_cast<size_t>(row) * 8U + col];
            const int n = snprintf(line + pos,
                                   static_cast<size_t>(sizeof(line) - pos),
                                   "%5.1f ", (double)t);
            if (n <= 0) {
                break;
            }
            pos += n;
            if (pos >= static_cast<int>(sizeof(line))) {
                break;
            }
        }
        ESP_LOGI(TAG, "r%u: %s", (unsigned)row, line);
    }
}

static void runDumpLoop(Amg8833& cam)
{
    ESP_LOGI(TAG, "Using addr 0x%02X — dumping frames every %lu ms",
             (unsigned)cam.addr7(), (unsigned long)kLoopMs);

    while (true) {
        float therm = 0.0f;
        amg8833_frame_t frame = {};
        if ((cam.readThermistor(therm) == ESP_OK) &&
            (cam.readFrame(frame) == ESP_OK)) {
            logFrame(frame, therm);
        } else {
            ESP_LOGW(TAG, "read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(kLoopMs));
    }
}

extern "C" void app_main(void)
{
    i2c_cfg_t bus_cfg = {};
    bus_cfg.port    = eI2cPort0;
    bus_cfg.sda_pin = kPinSda;
    bus_cfg.scl_pin = kPinScl;
    bus_cfg.clk_hz  = 400000U;

    I2C bus(bus_cfg);
    if (bus.init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }

    amg8833_cfg_t cfg = {};
    cfg.addr7 = amg8833_regs::kAddr7;
    cfg.fps   = amg8833_regs::eAmgFps10Hz;
    cfg.moving_average = true;

    Amg8833 cam_primary(bus, cfg);
    if (cam_primary.init() == ESP_OK) {
        runDumpLoop(cam_primary);
        return;
    }

    ESP_LOGW(TAG, "init @0x%02X failed; trying 0x%02X",
             (unsigned)amg8833_regs::kAddr7,
             (unsigned)amg8833_regs::kAddr7Alt);

    cfg.addr7 = amg8833_regs::kAddr7Alt;
    Amg8833 cam_alt(bus, cfg);
    if (cam_alt.init() != ESP_OK) {
        ESP_LOGE(TAG, "AMG8833 init failed on both addresses");
        return;
    }

    runDumpLoop(cam_alt);
}
