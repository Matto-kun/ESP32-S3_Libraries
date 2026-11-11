/**
 * @file        qmc5883p_example.cpp
 * @brief       Poll a QMC5883P and log raw counts and gauss
 * @author      Mat RB
 * @date        2026-11-10
 *
 * @details
 *   Pins stay SDA GPIO 8 and SCL GPIO 9. Internal pull-ups stay off.
 *   The 7-bit address is 0x2C. app_main does not return.
 */

#include "i2c_api.hpp"
#include "sensors/qmc5883p_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "QMC_EX";

static constexpr int8_t kPinSda = 8;
static constexpr int8_t kPinScl = 9;
static constexpr uint32_t kClkHz = 100000U;
static constexpr uint32_t kRetryMs = 500U;
static constexpr uint32_t kReadPeriodMs = 200U;

static void readLoop(Qmc5883p& mag)
{
    ESP_LOGI(TAG, "Reading addr7=0x%02X (%lu ms period)",
             static_cast<unsigned>(qmc5883p_regs::kAddr7),
             static_cast<unsigned long>(kReadPeriodMs));

    const uint16_t lsb = qmc5883p_regs::lsbPerGauss(qmc5883p_regs::eQmcRange8Ga);
    const float scale = 1.0f / static_cast<float>(lsb);

    while (true) {
        qmc5883p_raw_t raw = {};
        const esp_err_t err = mag.readRaw(raw);
        if ((err == ESP_OK) || (err == ESP_ERR_INVALID_RESPONSE)) {
            ESP_LOGI(TAG, "raw X=%d Y=%d Z=%d | gauss X=%.3f Y=%.3f Z=%.3f",
                     static_cast<int>(raw.x),
                     static_cast<int>(raw.y),
                     static_cast<int>(raw.z),
                     static_cast<double>(static_cast<float>(raw.x) * scale),
                     static_cast<double>(static_cast<float>(raw.y) * scale),
                     static_cast<double>(static_cast<float>(raw.z) * scale));
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(kReadPeriodMs));
    }
}

extern "C" void app_main(void)
{
    i2c_cfg_t bus_cfg = {};
    bus_cfg.port = eI2cPort0;
    bus_cfg.sda_pin = kPinSda;
    bus_cfg.scl_pin = kPinScl;
    bus_cfg.clk_hz = kClkHz;
    bus_cfg.enable_internal_pullup = false;

    I2C bus(bus_cfg);
    while (bus.init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        vTaskDelay(pdMS_TO_TICKS(kRetryMs));
    }

    Qmc5883p mag(bus);
    while (mag.init() != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883P init failed");
        vTaskDelay(pdMS_TO_TICKS(kRetryMs));
    }

    ESP_LOGI(TAG, "QMC5883P init ok SDA=%d SCL=%d",
             static_cast<int>(kPinSda),
             static_cast<int>(kPinScl));
    readLoop(mag);
}
