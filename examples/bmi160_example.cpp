/**
 * @file        bmi160_example.cpp
 * @brief       Poll BMI160 accel/gyro over I2C and log samples
 * @author      Mat RB
 * @date        2026-10-28
 *
 * @details
 *   Pins from proto_esp32_s3/docs/pinout_gpio.json i2c2 (example-local):
 *   Update after bring-up if SDA/SCL order differs on the board.
 *   CHIP_ID is 0xD1; 7-bit I2C address is 0x68 or 0x69.
 */

#include "i2c_api.hpp"
#include "sensors/bmi160_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "BMI_EX";

/* Confirmed on proto: SDA=9, SCL=11, addr7=0x69 (CHIP_ID 0xD1). */
static constexpr int8_t  kPinSda = 9;
static constexpr int8_t  kPinScl = 11;
static constexpr uint8_t kAddr7  = bmi160_regs::kAddr7SdoHigh;
static constexpr uint32_t kClkHz = 400000U;
static constexpr uint32_t kPollMs = 200U;

extern "C" void app_main(void)
{
    i2c_cfg_t bus_cfg = {};
    bus_cfg.port    = eI2cPort1;
    bus_cfg.sda_pin = kPinSda;
    bus_cfg.scl_pin = kPinScl;
    bus_cfg.clk_hz  = kClkHz;

    I2C bus(bus_cfg);
    if (bus.init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }

    bmi160_cfg_t imu_cfg = {};
    imu_cfg.addr7 = kAddr7;

    Bmi160 imu(bus, imu_cfg);
    if (imu.init() != ESP_OK) {
        ESP_LOGE(TAG, "BMI160 init failed");
        return;
    }

    ESP_LOGI(TAG, "Reading IMU every %lu ms", (unsigned long)kPollMs);

    while (true) {
        bmi160_vec3_t acc = {};
        bmi160_vec3_t gyr = {};
        if (imu.readImu(acc, gyr) == ESP_OK) {
            ESP_LOGI(TAG,
                     "acc[g]=%.3f,%.3f,%.3f  gyr[dps]=%.2f,%.2f,%.2f",
                     acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
        } else {
            ESP_LOGW(TAG, "readImu failed");
        }
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
    }
}
