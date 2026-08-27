/**
 * @file        spi_api.cpp
 * @brief       SPI master implementation for ESP32-S3 (GP-SPI hosts)
 * @author      Mat RB
 * @date        2026-08-12
 * @version     1.1.0
 *
 * @details
 *   Implements the SPI class defined in spi_api.hpp. All ESP-IDF peripheral
 *   types (driver/spi_master.h) are confined to this translation unit; the
 *   public header exposes only esp_err_t and project-owned types.
 *
 *   Bus path     — spi_bus_initialize() / spi_bus_free() with per-host refcount
 *   Device path  — spi_bus_add_device() / spi_bus_remove_device()
 *   Transfer path— spi_device_transmit() (blocking, task context only)
 *   CS            — not handled here; caller uses gpio_api (spics_io_num = -1)
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

/* ── Project Includes ────────────────────────────────────────────────────── */
#include "spi_api.hpp"

/* ── Framework Includes ──────────────────────────────────────────────────── */
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_check.h"

/* ── Module-local Constants ──────────────────────────────────────────────── */
static const char* TAG = "SPI";

/// Number of supported GP-SPI hosts (SPI2, SPI3).
static constexpr size_t kSpiHostCount = 2U;

/// Dummy TX byte clocked during read()-only transfers.
static constexpr uint8_t kSpiReadDummyTx = 0xFFU;

/* ── Per-host bus refcount (shared across SPI instances on the same host) ── */
static uint8_t s_bus_refcount[kSpiHostCount] = {0U, 0U};

/* ── File-scope Type Mapping Helpers ─────────────────────────────────────── */

/**
 * @brief Map project spi_host_t to ESP-IDF spi_host_device_t.
 */
static spi_host_device_t toEspHost(spi_host_t host)
{
    switch (host) {
        case eSpiHost3: return SPI3_HOST;
        case eSpiHost2: /* fall-through */
        default:        return SPI2_HOST;
    }
}

/**
 * @brief Index into s_bus_refcount for a project host enum.
 */
static size_t hostIndex(spi_host_t host)
{
    return (host == eSpiHost3) ? 1U : 0U;
}

/**
 * @brief Map project spi_bit_order_t to ESP-IDF device flags bit.
 *
 * @return SPI_DEVICE_BIT_LSBFIRST or 0.
 */
static uint32_t toEspBitOrderFlags(spi_bit_order_t order)
{
    return (order == eSpiBitOrderLsb) ? SPI_DEVICE_BIT_LSBFIRST : 0U;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Constructors / Destructor
   ═══════════════════════════════════════════════════════════════════════════ */

SPI::SPI(const spi_cfg_t& config)
    : m_config(config)
{
}

SPI::SPI(spi_host_t host,
         int8_t mosi_pin,
         int8_t miso_pin,
         int8_t sclk_pin,
         uint32_t clock_hz)
{
    m_config.host     = host;
    m_config.mosi_pin = mosi_pin;
    m_config.miso_pin = miso_pin;
    m_config.sclk_pin = sclk_pin;
    m_config.clock_hz = clock_hz;
    // Remaining fields use spi_cfg_t default member initialisers.
}

SPI::~SPI()
{
    if (m_inited) {
        (void)deinit();
    }
}


/* ═══════════════════════════════════════════════════════════════════════════
   Core
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t SPI::init()
{
    if (m_inited) {
        ESP_LOGW(TAG, "Host %d already inited", (int)m_config.host);
        return ESP_OK;
    }

    if (m_config.sclk_pin == kSpiPinUnused) {
        ESP_LOGE(TAG, "SCLK pin is required");
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.clock_hz == 0U) {
        ESP_LOGE(TAG, "clock_hz must be > 0");
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.queue_size == 0U) {
        ESP_LOGE(TAG, "queue_size must be > 0");
        return ESP_ERR_INVALID_ARG;
    }

    const spi_host_device_t esp_host = toEspHost(m_config.host);
    const size_t            idx      = hostIndex(m_config.host);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num     = m_config.mosi_pin;
    bus_cfg.miso_io_num     = m_config.miso_pin;
    bus_cfg.sclk_io_num     = m_config.sclk_pin;
    bus_cfg.quadwp_io_num   = -1;
    bus_cfg.quadhd_io_num   = -1;
    bus_cfg.max_transfer_sz = m_config.max_transfer;

    if (s_bus_refcount[idx] == 0U) {
        /* DMA enabled; async multi-buffer TX queue deferred. */
        ESP_RETURN_ON_ERROR(
            spi_bus_initialize(esp_host, &bus_cfg, SPI_DMA_CH_AUTO),
            TAG, "Host %d spi_bus_initialize failed: %s",
            (int)m_config.host, esp_err_to_name(err_rc_));
        m_bus_owned = true;
        s_bus_refcount[idx] = 1U;
    } else {
        // Bus already up (another SPI instance on this host).
        s_bus_refcount[idx]++;
        m_bus_owned = true;
        ESP_LOGI(TAG, "Host %d bus shared (refcount=%u)",
                 (int)m_config.host, (unsigned)s_bus_refcount[idx]);
    }

    // No hardware CS — caller owns CS via gpio_api (one or many slaves).
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = static_cast<int>(m_config.clock_hz);
    dev_cfg.mode           = static_cast<uint8_t>(m_config.mode);
    dev_cfg.spics_io_num   = -1;
    dev_cfg.queue_size     = m_config.queue_size;
    dev_cfg.flags          = toEspBitOrderFlags(m_config.bit_order);

    spi_device_handle_t handle = nullptr;
    esp_err_t err = spi_bus_add_device(esp_host, &dev_cfg, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Host %d spi_bus_add_device failed: %s",
                 (int)m_config.host, esp_err_to_name(err));
        if (m_bus_owned) {
            s_bus_refcount[idx]--;
            m_bus_owned = false;
            if (s_bus_refcount[idx] == 0U) {
                (void)spi_bus_free(esp_host);
            }
        }
        return err;
    }

    m_device = handle;
    m_inited = true;
    ESP_LOGI(TAG, "Host %d inited (clk=%lu Hz, mode=%d)",
             (int)m_config.host,
             (unsigned long)m_config.clock_hz,
             (int)m_config.mode);
    return ESP_OK;
}

esp_err_t SPI::deinit()
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Host %d not inited (deinit)", (int)m_config.host);
        return ESP_OK;
    }

    if (m_device != nullptr) {
        ESP_RETURN_ON_ERROR(
            spi_bus_remove_device(static_cast<spi_device_handle_t>(m_device)),
            TAG, "Host %d spi_bus_remove_device failed: %s",
            (int)m_config.host, esp_err_to_name(err_rc_));
        m_device = nullptr;
    }

    if (m_bus_owned) {
        const size_t            idx      = hostIndex(m_config.host);
        const spi_host_device_t esp_host = toEspHost(m_config.host);

        if (s_bus_refcount[idx] > 0U) {
            s_bus_refcount[idx]--;
        }
        m_bus_owned = false;

        if (s_bus_refcount[idx] == 0U) {
            ESP_RETURN_ON_ERROR(
                spi_bus_free(esp_host),
                TAG, "Host %d spi_bus_free failed: %s",
                (int)m_config.host, esp_err_to_name(err_rc_));
        }
    }

    m_inited = false;
    ESP_LOGI(TAG, "Host %d deinited", (int)m_config.host);
    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Private helpers
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t SPI::transmit(const uint8_t* tx, uint8_t* rx, size_t len)
{
    if (!m_inited || m_device == nullptr) {
        ESP_LOGW(TAG, "Host %d not inited (transmit)", (int)m_config.host);
        return ESP_ERR_INVALID_STATE;
    }

    if (len == 0U) {
        return ESP_OK;
    }

    if ((tx == nullptr) && (rx == nullptr)) {
        ESP_LOGE(TAG, "Host %d transmit requires tx and/or rx buffer", (int)m_config.host);
        return ESP_ERR_INVALID_ARG;
    }

    if (static_cast<int>(len) > m_config.max_transfer && m_config.max_transfer > 0) {
        ESP_LOGE(TAG, "Host %d len %u exceeds max_transfer %d",
                 (int)m_config.host, (unsigned)len, m_config.max_transfer);
        return ESP_ERR_INVALID_SIZE;
    }

    spi_transaction_t t = {};
    t.length    = len * 8U;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    ESP_RETURN_ON_ERROR(
        spi_device_transmit(static_cast<spi_device_handle_t>(m_device), &t),
        TAG, "Host %d spi_device_transmit failed: %s",
        (int)m_config.host, esp_err_to_name(err_rc_));

    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   I/O
   ═══════════════════════════════════════════════════════════════════════════ */

esp_err_t SPI::write(const uint8_t* data, size_t len)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Host %d not inited (write)", (int)m_config.host);
        return ESP_ERR_INVALID_STATE;
    }

    if (data == nullptr && len > 0U) {
        ESP_LOGE(TAG, "Host %d write null buffer", (int)m_config.host);
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.mosi_pin == kSpiPinUnused && len > 0U) {
        ESP_LOGE(TAG, "Host %d write requires MOSI pin", (int)m_config.host);
        return ESP_ERR_NOT_SUPPORTED;
    }

    return transmit(data, nullptr, len);
}

esp_err_t SPI::read(uint8_t* data, size_t len)
{
    if (!m_inited) {
        ESP_LOGW(TAG, "Host %d not inited (read)", (int)m_config.host);
        return ESP_ERR_INVALID_STATE;
    }

    if (data == nullptr && len > 0U) {
        ESP_LOGE(TAG, "Host %d read null buffer", (int)m_config.host);
        return ESP_ERR_INVALID_ARG;
    }

    if (m_config.miso_pin == kSpiPinUnused && len > 0U) {
        ESP_LOGE(TAG, "Host %d read requires MISO pin", (int)m_config.host);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (len == 0U) {
        return ESP_OK;
    }

    // Clock out dummy bytes while filling rx. Stack buffer for small reads;
    // chunk larger transfers to avoid large stack use / heap.
    static constexpr size_t kChunk = 64U;
    uint8_t dummy[kChunk];
    for (size_t i = 0U; i < kChunk; i++) {
        dummy[i] = kSpiReadDummyTx;
    }

    size_t offset = 0U;
    while (offset < len) {
        const size_t chunk = ((len - offset) > kChunk) ? kChunk : (len - offset);
        const esp_err_t err = transmit(dummy, data + offset, chunk);
        if (err != ESP_OK) {
            return err;
        }
        offset += chunk;
    }

    return ESP_OK;
}


/* ═══════════════════════════════════════════════════════════════════════════
   Status
   ═══════════════════════════════════════════════════════════════════════════ */

const spi_cfg_t& SPI::getConfig() const
{
    return m_config;
}

bool SPI::isInited() const
{
    return m_inited;
}
