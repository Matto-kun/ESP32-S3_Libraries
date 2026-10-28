/**
 * @file        sd_api.cpp
 * @brief       SPI microSD (SDSPI + FatFS) implementation
 * @author      Mat RB
 * @date        2026-09-06
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#include "sd_api.hpp"

#include <string.h>

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char* TAG = "SD";

static constexpr int kSdMaxTransferBytes = 4000;
static constexpr uint64_t kBytesPerMiB = 1024ULL * 1024ULL;
static constexpr uint64_t kBytesPerGiB = 1024ULL * 1024ULL * 1024ULL;

SdCard::SdCard(const sd_cfg_t& cfg)
    : m_cfg(cfg)
{
}

SdCard::~SdCard()
{
    (void)deinit();
}

int SdCard::toEspHost(sd_spi_host_t host)
{
    switch (host) {
    case eSdSpiHost2:
        return static_cast<int>(SPI2_HOST);
    case eSdSpiHost3:
        return static_cast<int>(SPI3_HOST);
    default:
        return -1;
    }
}

esp_err_t SdCard::validateCfg() const
{
    if (toEspHost(m_cfg.host) < 0) {
        ESP_LOGE(TAG, "Invalid SPI host");
        return ESP_ERR_INVALID_ARG;
    }
    if ((m_cfg.mosi_pin < 0) || (m_cfg.miso_pin < 0) ||
        (m_cfg.sclk_pin < 0) || (m_cfg.cs_pin < 0)) {
        ESP_LOGE(TAG, "MOSI/MISO/SCLK/CS pins required");
        return ESP_ERR_INVALID_ARG;
    }
    if ((m_cfg.mount_point[0] == '\0') ||
        (strnlen(m_cfg.mount_point, kSdMountPointLen) >= kSdMountPointLen)) {
        ESP_LOGE(TAG, "Invalid mount_point");
        return ESP_ERR_INVALID_ARG;
    }
    if (m_cfg.max_open_files <= 0) {
        ESP_LOGE(TAG, "max_open_files must be > 0");
        return ESP_ERR_INVALID_ARG;
    }
    if (m_cfg.max_freq_khz == 0U) {
        ESP_LOGE(TAG, "max_freq_khz must be > 0");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t SdCard::init()
{
    if (m_mounted) {
        ESP_LOGW(TAG, "Already mounted at %s", m_cfg.mount_point);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(validateCfg(), TAG, "validateCfg");

    const int host_id = toEspHost(m_cfg.host);
    ESP_LOGI(TAG,
             "SDSPI host=%d MOSI=%d MISO=%d SCLK=%d CS=%d freq=%lu kHz mount=%s",
             host_id,
             (int)m_cfg.mosi_pin,
             (int)m_cfg.miso_pin,
             (int)m_cfg.sclk_pin,
             (int)m_cfg.cs_pin,
             (unsigned long)m_cfg.max_freq_khz,
             m_cfg.mount_point);

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = host_id;
    host.max_freq_khz = static_cast<int>(m_cfg.max_freq_khz);

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = m_cfg.mosi_pin;
    bus_cfg.miso_io_num = m_cfg.miso_pin;
    bus_cfg.sclk_io_num = m_cfg.sclk_pin;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = kSdMaxTransferBytes;

    esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(host_id),
                                       &bus_cfg,
                                       SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }
    m_bus_owned = true;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.host_id = static_cast<spi_host_device_t>(host_id);
    slot_cfg.gpio_cs = static_cast<gpio_num_t>(m_cfg.cs_pin);
    if (m_cfg.cd_pin >= 0) {
        slot_cfg.gpio_cd = static_cast<gpio_num_t>(m_cfg.cd_pin);
    }
    if (m_cfg.wp_pin >= 0) {
        slot_cfg.gpio_wp = static_cast<gpio_num_t>(m_cfg.wp_pin);
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {};
    mount_cfg.format_if_mount_failed = m_cfg.format_if_mount_failed;
    mount_cfg.max_files = m_cfg.max_open_files;
    mount_cfg.allocation_unit_size = m_cfg.allocation_unit_size;

    sdmmc_card_t* card = nullptr;
    err = esp_vfs_fat_sdspi_mount(m_cfg.mount_point,
                                  &host,
                                  &slot_cfg,
                                  &mount_cfg,
                                  &card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Mount failed (%s). Check card seating and pull-ups on SPI lines.",
                 esp_err_to_name(err));
        (void)spi_bus_free(static_cast<spi_host_device_t>(host_id));
        m_bus_owned = false;
        m_card = nullptr;
        return err;
    }

    m_card = card;
    m_mounted = true;

    sd_info_t info = {};
    if (getInfo(info) == ESP_OK) {
        const double mib = static_cast<double>(info.capacity_bytes) /
                           static_cast<double>(kBytesPerMiB);
        const double gib = static_cast<double>(info.capacity_bytes) /
                           static_cast<double>(kBytesPerGiB);
        ESP_LOGI(TAG,
                 "Mounted %s name='%s' capacity=%.1f MiB (%.2f GiB) sector=%lu count=%lu",
                 m_cfg.mount_point,
                 info.name,
                 mib,
                 gib,
                 (unsigned long)info.sector_size,
                 (unsigned long)info.sector_count);
    } else {
        ESP_LOGI(TAG, "Mounted %s", m_cfg.mount_point);
    }

    return ESP_OK;
}

esp_err_t SdCard::deinit()
{
    if (!m_mounted && !m_bus_owned) {
        return ESP_OK;
    }

    const int host_id = toEspHost(m_cfg.host);
    esp_err_t err = ESP_OK;

    if (m_mounted) {
        sdmmc_card_t* card = static_cast<sdmmc_card_t*>(m_card);
        err = esp_vfs_fat_sdcard_unmount(m_cfg.mount_point, card);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Unmount failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Unmounted %s", m_cfg.mount_point);
        }
        m_mounted = false;
        m_card = nullptr;
    }

    if (m_bus_owned && (host_id >= 0)) {
        const esp_err_t bus_err =
            spi_bus_free(static_cast<spi_host_device_t>(host_id));
        if (bus_err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_free failed: %s", esp_err_to_name(bus_err));
            if (err == ESP_OK) {
                err = bus_err;
            }
        }
        m_bus_owned = false;
    }

    return err;
}

bool SdCard::isMounted() const
{
    return m_mounted;
}

const char* SdCard::mountPoint() const
{
    return m_cfg.mount_point;
}

esp_err_t SdCard::getInfo(sd_info_t& out) const
{
    if (!m_mounted || (m_card == nullptr)) {
        return ESP_ERR_INVALID_STATE;
    }

    const sdmmc_card_t* card = static_cast<const sdmmc_card_t*>(m_card);
    memset(&out, 0, sizeof(out));

    /* CID product name is not always NUL-terminated. */
    const size_t cid_len = strnlen(card->cid.name, sizeof(card->cid.name));
    const size_t copy_len =
        (cid_len < (kSdCardNameLen - 1U)) ? cid_len : (kSdCardNameLen - 1U);
    memcpy(out.name, card->cid.name, copy_len);
    out.name[copy_len] = '\0';

    out.sector_size = static_cast<uint32_t>(card->csd.sector_size);
    out.sector_count = static_cast<uint32_t>(card->csd.capacity);
    out.capacity_bytes =
        static_cast<uint64_t>(out.sector_count) *
        static_cast<uint64_t>(out.sector_size);

    return ESP_OK;
}

esp_err_t SdCard::format()
{
    if (!m_mounted || (m_card == nullptr)) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG,
             "Formatting %s (allocation_unit=%u). Large cards (e.g. 16 GB) "
             "can take several minutes over SPI.",
             m_cfg.mount_point,
             (unsigned)m_cfg.allocation_unit_size);

    sdmmc_card_t* card = static_cast<sdmmc_card_t*>(m_card);
    const esp_err_t err =
        esp_vfs_fat_sdcard_format(m_cfg.mount_point, card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "format failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Format complete on %s", m_cfg.mount_point);
    return ESP_OK;
}
