/**
 * @file        hmc5883l_example.cpp
 * @brief       Poll a magnetometer over I2C and log raw + Gauss readings
 * @author      Mat RB
 * @date        2026-08-19
 *
 * @details
 *   Pins stay SDA GPIO 8 and SCL GPIO 9. DRDY GPIO 10 is stored and not used.
 *   Internal pull-ups stay off. The module has its own pull-ups.
 *   The loop probes 7-bit addresses 0x1E, 0x3C, 0x0D, and 0x2C.
 *   0x3C is the 8-bit write address of 7-bit 0x1E. 0x0D is the QMC5883L clone.
 *   app_main does not return.
 */

#include "i2c_api.hpp"
#include "sensors/hmc5883l_api.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HMC_EX";

static constexpr int8_t kPinSda = 8;
static constexpr int8_t kPinScl = 9;
static constexpr int8_t kPinDrdy = 10;
static constexpr uint32_t kClkHz = 100000U;
static constexpr uint32_t kProbeTimeoutMs = 50U;
static constexpr uint32_t kInitRetryMs = 500U;
static constexpr uint32_t kReadPeriodMs = 500U;

static constexpr uint8_t kAddrs[] = {
    0x1EU,
    0x3CU,
    0x0DU,
    0x2CU,
};

static void logLines(void)
{
    const int sda_level = gpio_get_level(static_cast<gpio_num_t>(kPinSda));
    const int scl_level = gpio_get_level(static_cast<gpio_num_t>(kPinScl));
    ESP_LOGI(TAG, "SDA %d SCL %d levels SDA=%d SCL=%d DRDY %d clk=%lu",
             static_cast<int>(kPinSda),
             static_cast<int>(kPinScl),
             sda_level,
             scl_level,
             static_cast<int>(kPinDrdy),
             static_cast<unsigned long>(kClkHz));
}

static void readLoop(Hmc5883l& mag, uint8_t addr7)
{
    ESP_LOGI(TAG, "Reading addr7=0x%02X (%lu ms period)",
             static_cast<unsigned>(addr7),
             static_cast<unsigned long>(kReadPeriodMs));

    while (true) {
        hmc5883l_raw_t raw = {};
        hmc5883l_gauss_t gauss = {};

        const esp_err_t err_raw = mag.readRaw(raw);
        const esp_err_t err_g = mag.readGauss(gauss);

        if ((err_raw == ESP_OK) && (err_g == ESP_OK)) {
            ESP_LOGI(TAG, "raw X=%d Y=%d Z=%d | gauss X=%.3f Y=%.3f Z=%.3f",
                     static_cast<int>(raw.x), static_cast<int>(raw.y), static_cast<int>(raw.z),
                     static_cast<double>(gauss.x), static_cast<double>(gauss.y),
                     static_cast<double>(gauss.z));
        } else {
            ESP_LOGW(TAG, "read failed raw=%s gauss=%s",
                     esp_err_to_name(err_raw), esp_err_to_name(err_g));
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
        vTaskDelay(pdMS_TO_TICKS(kInitRetryMs));
    }

    while (true) {
        logLines();
        for (uint8_t addr7 : kAddrs) {
            const esp_err_t ack = bus.probe(addr7, kProbeTimeoutMs);
            if (ack != ESP_OK) {
                ESP_LOGI(TAG, "probe addr7=0x%02X ACK=no", static_cast<unsigned>(addr7));
                vTaskDelay(pdMS_TO_TICKS(kInitRetryMs));
                continue;
            }

            ESP_LOGI(TAG, "probe addr7=0x%02X ACK=yes", static_cast<unsigned>(addr7));
            Hmc5883l mag(bus, addr7);
            if (mag.init() == ESP_OK) {
                ESP_LOGI(TAG, "HMC id H43 at addr7=0x%02X", static_cast<unsigned>(addr7));
                readLoop(mag, addr7);
            }
            ESP_LOGW(TAG, "addr7=0x%02X ACK, HMC id check failed",
                     static_cast<unsigned>(addr7));
            vTaskDelay(pdMS_TO_TICKS(kInitRetryMs));
        }
    }
}
