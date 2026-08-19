/**
 * @file        spi_api.hpp
 * @brief       SPI master driver class for ESP32-S3 (GP-SPI hosts)
 * @author      Mat RB
 * @date        2026-08-12
 * @version     1.1.0
 *
 * @details
 *   Wraps the ESP-IDF SPI master driver behind a unified C++ interface.
 *   Supports SPI2 and SPI3 hosts (SPI1 is reserved for flash on ESP32-S3).
 *   Chip-select is intentionally not part of this module — drive one or more
 *   CS lines with gpio_api (or raw GPIO) so multi-slave buses stay flexible.
 *   All ESP-IDF peripheral types (driver/spi_master.h) are isolated to
 *   spi_api.cpp; only esp_err_t crosses this header boundary.
 *
 *   Typical framing (CS via GPIO):
 *   @code
 *     cs.write(false);          // assert active-low CS
 *     spi.write(tx, tx_len);
 *     spi.read(rx, rx_len);
 *     cs.write(true);           // deassert
 *   @endcode
 *
 * @note read() / write() block the calling task until the transfer completes
 *       (spi_device_transmit). Do not call from an ISR.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef SPI_API_HPP
#define SPI_API_HPP

/* ── System Includes ─────────────────────────────────────────────────────── */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ── Framework Includes ──────────────────────────────────────────────────── */
#include "esp_err.h"

/* ── Constants ───────────────────────────────────────────────────────────── */

/// Pin unused / not connected — pass for optional MISO or MOSI.
static constexpr int8_t kSpiPinUnused = -1;

/// Default SPI clock when not specified (safe for most slaves).
static constexpr uint32_t kSpiDefaultClockHz = 1000000U;

/// Default max transfer size in bytes (0 = ESP-IDF driver default).
static constexpr int kSpiDefaultMaxTransfer = 4092;

/* ── Enumerations ────────────────────────────────────────────────────────── */

/**
 * @brief GP-SPI host selector (ESP32-S3).
 *
 * @note SPI1 is reserved for flash and is not exposed.
 */
typedef enum {
    eSpiHost2 = 0,   ///< SPI2_HOST — general-purpose
    eSpiHost3 = 1,   ///< SPI3_HOST — general-purpose
} spi_host_t;

/**
 * @brief SPI clock polarity / phase (Motorola modes 0–3).
 */
typedef enum {
    eSpiMode0 = 0,   ///< CPOL=0, CPHA=0
    eSpiMode1 = 1,   ///< CPOL=0, CPHA=1
    eSpiMode2 = 2,   ///< CPOL=1, CPHA=0
    eSpiMode3 = 3,   ///< CPOL=1, CPHA=1
} spi_mode_t;

/**
 * @brief SPI bit order on the wire.
 */
typedef enum {
    eSpiBitOrderMsb = 0,   ///< MSB first (default)
    eSpiBitOrderLsb = 1,   ///< LSB first
} spi_bit_order_t;

/* ── Structures ──────────────────────────────────────────────────────────── */

/**
 * @brief SPI master driver configuration.
 *
 * Pin numbers and host must be set by the caller — no board defaults.
 * Unused lines (e.g. MISO on a write-only bus) use kSpiPinUnused.
 * CS is not configured here; manage CS with gpio_api outside this module.
 *
 * Example (convenience constructor):
 * @code
 *   SPI bus(eSpiHost2, 35, kSpiPinUnused, 36, 1000000U);
 *   bus.init();
 * @endcode
 *
 * Example (full config):
 * @code
 *   spi_cfg_t cfg = {};
 *   cfg.host      = eSpiHost2;
 *   cfg.mosi_pin  = 35;
 *   cfg.miso_pin  = kSpiPinUnused;
 *   cfg.sclk_pin  = 36;
 *   cfg.clock_hz  = 1000000U;
 *   cfg.mode      = eSpiMode0;
 *   SPI bus(cfg);
 *   bus.init();
 * @endcode
 */
typedef struct {
    spi_host_t      host           = eSpiHost2;              ///< GP-SPI host
    int8_t          mosi_pin       = kSpiPinUnused;          ///< MOSI GPIO (or unused)
    int8_t          miso_pin       = kSpiPinUnused;          ///< MISO GPIO (or unused)
    int8_t          sclk_pin       = kSpiPinUnused;          ///< SCLK GPIO (required)
    uint32_t        clock_hz       = kSpiDefaultClockHz;     ///< SPI clock frequency in Hz
    spi_mode_t      mode           = eSpiMode0;              ///< SPI mode 0–3
    spi_bit_order_t bit_order      = eSpiBitOrderMsb;        ///< Bit order
    int             max_transfer   = kSpiDefaultMaxTransfer; ///< Max bytes per transaction
    uint8_t         queue_size     = 1U;                     ///< IDF transaction queue depth
} spi_cfg_t;

/* ── Class Definition ────────────────────────────────────────────────────── */

/**
 * @brief SPI master bus/device for ESP32-S3 (no chip-select ownership).
 *
 * Hardware is untouched until init() is called. The destructor performs safe
 * teardown (deinit) for initialised instances.
 *
 * @note CS must be framed by the caller (gpio_api). IDF hardware CS is left
 *       unused (spics_io_num = -1) so multiple slaves can share the bus.
 */
class SPI
{
public:
    explicit SPI(const spi_cfg_t& config);   ///< Full-config constructor; hardware untouched until init()
    SPI(spi_host_t host,
        int8_t mosi_pin,
        int8_t miso_pin,
        int8_t sclk_pin,
        uint32_t clock_hz = kSpiDefaultClockHz); ///< Convenience: mode 0, MSB first
    ~SPI();                                  ///< Calls deinit() if active

    // ── Core ──────────────────────────────────────────────────────────────────────
    esp_err_t init();    ///< spi_bus_initialize + add device
    esp_err_t deinit();  ///< remove device; free bus when last user

    // ── I/O ───────────────────────────────────────────────────────────────────────
    esp_err_t write(const uint8_t* data, size_t len); ///< TX only (rx discarded)
    esp_err_t read(uint8_t* data, size_t len);        ///< RX only (tx clocks 0xFF)

    // ── Status ────────────────────────────────────────────────────────────────────
    const spi_cfg_t& getConfig() const; ///< Returns current configuration struct
    bool             isInited()  const; ///< Returns m_inited

private:
    spi_cfg_t m_config    = {};
    bool      m_inited    = false;
    bool      m_bus_owned = false;   ///< True if this instance incremented the host bus refcount
    void*     m_device    = nullptr; ///< Opaque spi_device_handle_t

    esp_err_t transmit(const uint8_t* tx, uint8_t* rx, size_t len);
};

#endif /* SPI_API_HPP */
