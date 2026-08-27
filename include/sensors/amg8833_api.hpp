/**
 * @file        amg8833_api.hpp
 * @brief       Panasonic AMG8833 Grid-EYE 8x8 IR array driver
 * @author      Mat RB
 * @date        2026-08-26
 * @version     1.0.0
 *
 * @details
 *   Uses i2c_api for register access. Default 7-bit address 0x69 (AD0 high).
 *   Does not touch other I2C slaves (e.g. HMC5883L at 0x1E on a shared bus).
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef AMG8833_API_HPP
#define AMG8833_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "i2c_api.hpp"
#include "sensors/amg8833_regs.hpp"

typedef struct {
    uint8_t addr7 = amg8833_regs::kAddr7;
    amg8833_regs::FrameRate fps = amg8833_regs::eAmgFps10Hz;
    bool moving_average = true;
    uint32_t timeout_ms = amg8833_regs::kDefaultTimeoutMs;
} amg8833_cfg_t;

typedef struct {
    float pixels[amg8833_regs::kPixelCount];
} amg8833_frame_t;

class Amg8833
{
public:
    explicit Amg8833(I2C& bus, const amg8833_cfg_t& cfg = {});
    Amg8833(I2C& bus, uint8_t addr7);

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t readThermistor(float& celsius);
    esp_err_t readFrame(amg8833_frame_t& out);

    bool isInited() const;
    uint8_t addr7() const;

private:
    I2C&           m_bus;
    amg8833_cfg_t  m_cfg;
    bool           m_inited = false;

    esp_err_t writeReg(uint8_t reg, uint8_t val);
    esp_err_t readRegs(uint8_t reg, uint8_t* buf, size_t len);
    esp_err_t applyConfig();
};

#endif /* AMG8833_API_HPP */
