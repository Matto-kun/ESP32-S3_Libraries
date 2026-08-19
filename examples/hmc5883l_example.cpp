/**
 * @file        hmc5883l_example.cpp
 * @brief       Poll HMC5883L over I2C and log raw + Gauss readings
 * @author      Mat RB
 * @date        2026-08-19
 *
 * @details
 *   Pins from proto_esp32_s3/docs/pinout_gpio.json (example-local only):
 *   SDA = GPIO 9, SCL = GPIO 11
 */

#include "i2c_api.hpp"
#include "sensors/hmc5883l_api.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HMC_EX";

static constexpr int8_t kPinSda = 9;
static constexpr int8_t kPinScl = 11;

extern "C" void app_main(void)
{
    i2c_cfg_t bus_cfg = {};
    bus_cfg.port     = eI2cPort0;
    bus_cfg.sda_pin  = kPinSda;
    bus_cfg.scl_pin  = kPinScl;
    bus_cfg.clk_hz   = 400000U;

    I2C bus(bus_cfg);

    if (bus.init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }

    hmc5883l_cfg_t mag_cfg = {};
    mag_cfg.mode = hmc5883l_regs::eHmcModeContinuous;

    Hmc5883l mag(bus, mag_cfg);

    if (mag.init() != ESP_OK) {
        ESP_LOGE(TAG, "HMC5883L init failed");
        return;
    }

    ESP_LOGI(TAG, "Reading magnetometer (500 ms period)");

    while (true) {
        hmc5883l_raw_t raw = {};
        hmc5883l_gauss_t gauss = {};

        const esp_err_t err_raw = mag.readRaw(raw);
        const esp_err_t err_g = mag.readGauss(gauss);

        if ((err_raw == ESP_OK) && (err_g == ESP_OK)) {
            ESP_LOGI(TAG, "raw X=%d Y=%d Z=%d | gauss X=%.3f Y=%.3f Z=%.3f",
                     (int)raw.x, (int)raw.y, (int)raw.z,
                     (double)gauss.x, (double)gauss.y, (double)gauss.z);
        } else {
            ESP_LOGW(TAG, "read failed raw=%s gauss=%s",
                     esp_err_to_name(err_raw), esp_err_to_name(err_g));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
