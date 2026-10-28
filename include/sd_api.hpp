/**
 * @file        sd_api.hpp
 * @brief       SPI microSD mount helper (SDSPI + FatFS VFS) for ESP32-S3
 * @author      Mat RB
 * @date        2026-09-06
 * @version     1.0.0
 *
 * @details
 *   Owns SPI bus init and FatFS mount via ESP-IDF SDSPI. After init(), callers
 *   use POSIX I/O on mountPoint() (fopen/fread/fwrite/opendir). Does not wrap
 *   file operations. Native 4-bit SDMMC/SDIO is out of scope for SPI wiring.
 *
 * @note init()/deinit()/format()/getInfo() block the calling task. Do not call
 *       from an ISR. format() can take minutes on large cards (e.g. 16 GB).
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO)
 */

#ifndef SD_API_HPP
#define SD_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

static constexpr int8_t kSdPinUnused = -1;
static constexpr uint32_t kSdDefaultMaxFreqKhz = 20000U;
static constexpr size_t kSdDefaultAllocUnit = 16U * 1024U;
static constexpr size_t kSdMountPointLen = 16U;
static constexpr size_t kSdCardNameLen = 32U;

/**
 * @brief GP-SPI host for SDSPI (ESP32-S3: SPI1 is flash-reserved).
 */
typedef enum {
    eSdSpiHost2 = 0,   ///< SPI2_HOST
    eSdSpiHost3 = 1,   ///< SPI3_HOST
} sd_spi_host_t;

/**
 * @brief SDSPI + FatFS mount configuration (no board pin defaults).
 */
typedef struct {
    sd_spi_host_t host = eSdSpiHost2;
    int8_t  mosi_pin = kSdPinUnused;
    int8_t  miso_pin = kSdPinUnused;
    int8_t  sclk_pin = kSdPinUnused;
    int8_t  cs_pin   = kSdPinUnused;
    int8_t  cd_pin   = kSdPinUnused;
    int8_t  wp_pin   = kSdPinUnused;
    uint32_t max_freq_khz = kSdDefaultMaxFreqKhz;
    char    mount_point[kSdMountPointLen] = "/sdcard";
    bool    format_if_mount_failed = false;
    size_t  allocation_unit_size = kSdDefaultAllocUnit;
    int     max_open_files = 5;
} sd_cfg_t;

/**
 * @brief Card identity / capacity snapshot (from sdmmc_card_t).
 */
typedef struct {
    char     name[kSdCardNameLen] = {};
    uint64_t capacity_bytes = 0U;
    uint32_t sector_size = 0U;
    uint32_t sector_count = 0U;
} sd_info_t;

/**
 * @brief SPI microSD card: mount lifecycle, info, and optional FAT format.
 *
 * Hardware untouched until init(). Destructor calls deinit() if mounted.
 */
class SdCard
{
public:
    explicit SdCard(const sd_cfg_t& cfg = {});
    ~SdCard();

    esp_err_t init();
    esp_err_t deinit();

    bool isMounted() const;
    const char* mountPoint() const;

    esp_err_t getInfo(sd_info_t& out) const;

    /**
     * @brief Format the mounted volume as FAT (destructive).
     * @note Blocks a long time on large media; do not call from ISR.
     */
    esp_err_t format();

private:
    sd_cfg_t m_cfg;
    bool     m_mounted = false;
    bool     m_bus_owned = false;
    void*    m_card = nullptr;

    esp_err_t validateCfg() const;
    static int toEspHost(sd_spi_host_t host);
};

#endif /* SD_API_HPP */
