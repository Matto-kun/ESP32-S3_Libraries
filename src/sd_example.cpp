/**
 * @file        sd_example.cpp
 * @brief       UART tester for SPI microSD (SdCard mount + POSIX I/O)
 * @author      Mat RB
 * @date        2026-09-06
 *
 * @details
 *   Pins from proto_esp32_s3 pinout (example-local only):
 *   MOSI=35, MISO=37, SCK=36, CS=15 on SPI2.
 *   Expects a ~16 GB FAT card. Does not format the card.
 */

#include "sd_api.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char* TAG = "SD_EX";

static constexpr int8_t kPinMosi = 35;
static constexpr int8_t kPinMiso = 37;
static constexpr int8_t kPinSclk = 36;
static constexpr int8_t kPinCs = 15;
static constexpr uint32_t kHeartbeatMs = 2000U;
static constexpr uint64_t kBytesPerMiB = 1024ULL * 1024ULL;
static constexpr uint64_t kBytesPerGiB = 1024ULL * 1024ULL * 1024ULL;

static constexpr char kTestRelPath[] = "hello.txt";
static constexpr char kTestPayload[] = "ESP32-S3 SDSPI hello (16GB card test)\n";

static void buildAbsPath(const SdCard& sd, const char* rel, char* out, size_t out_len)
{
    snprintf(out, out_len, "%s/%s", sd.mountPoint(), rel);
}

static esp_err_t posixWriteTest(const char* path)
{
    ESP_LOGI(TAG, "Writing %s (%u bytes)", path, (unsigned)(sizeof(kTestPayload) - 1U));

    FILE* f = fopen(path, "wb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "fopen(wb) failed for %s", path);
        return ESP_FAIL;
    }

    const size_t n = fwrite(kTestPayload, 1U, sizeof(kTestPayload) - 1U, f);
    (void)fflush(f);
    (void)fsync(fileno(f));
    (void)fclose(f);

    if (n != (sizeof(kTestPayload) - 1U)) {
        ESP_LOGE(TAG, "fwrite short write: %u", (unsigned)n);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Wrote %u bytes OK", (unsigned)n);
    return ESP_OK;
}

static esp_err_t posixReadTest(const char* path)
{
    ESP_LOGI(TAG, "Reading %s", path);

    FILE* f = fopen(path, "rb");
    if (f == nullptr) {
        ESP_LOGE(TAG, "fopen(rb) failed for %s", path);
        return ESP_FAIL;
    }

    char buf[96] = {};
    const size_t n = fread(buf, 1U, sizeof(buf) - 1U, f);
    (void)fclose(f);

    buf[n] = '\0';
    char* nl = strchr(buf, '\n');
    if (nl != nullptr) {
        *nl = '\0';
    }

    ESP_LOGI(TAG, "Read %u bytes: '%s'", (unsigned)n, buf);
    return ESP_OK;
}

static esp_err_t posixListRoot(const SdCard& sd)
{
    ESP_LOGI(TAG, "Listing %s", sd.mountPoint());

    DIR* dir = opendir(sd.mountPoint());
    if (dir == nullptr) {
        ESP_LOGE(TAG, "opendir failed for %s", sd.mountPoint());
        return ESP_FAIL;
    }

    uint32_t count = 0U;
    while (true) {
        errno = 0;
        const struct dirent* ent = readdir(dir);
        if (ent == nullptr) {
            break;
        }
        ESP_LOGI(TAG, "  [%lu] %s", (unsigned long)count, ent->d_name);
        count++;
    }

    if (errno != 0) {
        ESP_LOGE(TAG, "readdir error errno=%d", errno);
        (void)closedir(dir);
        return ESP_FAIL;
    }

    (void)closedir(dir);
    ESP_LOGI(TAG, "Listed %lu entries", (unsigned long)count);
    return ESP_OK;
}

static void heartbeatLoop(const char* path)
{
    ESP_LOGI(TAG, "Heartbeat every %lu ms (stat %s)",
             (unsigned long)kHeartbeatMs, path);

    while (true) {
        struct stat st = {};
        if (stat(path, &st) == 0) {
            ESP_LOGI(TAG, "alive: %s size=%ld", path, (long)st.st_size);
        } else {
            ESP_LOGW(TAG, "stat failed for %s", path);
        }
        vTaskDelay(pdMS_TO_TICKS(kHeartbeatMs));
    }
}

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "=== SDSPI example (expect ~16 GB FAT card) ===");
    ESP_LOGI(TAG, "Pins: MOSI=%d MISO=%d SCK=%d CS=%d",
             (int)kPinMosi, (int)kPinMiso, (int)kPinSclk, (int)kPinCs);
    ESP_LOGI(TAG,
             "Note: if ST7789 CS is tied low on shared MOSI/SCK, SD mount will fail");

    sd_cfg_t cfg = {};
    cfg.host = eSdSpiHost2;
    cfg.mosi_pin = kPinMosi;
    cfg.miso_pin = kPinMiso;
    cfg.sclk_pin = kPinSclk;
    cfg.cs_pin = kPinCs;
    cfg.format_if_mount_failed = false;
    cfg.max_freq_khz = 20000U;

    SdCard sd(cfg);
    const esp_err_t mount_err = sd.init();
    if (mount_err != ESP_OK) {
        ESP_LOGE(TAG,
                 "init failed: %s — check seating and SPI pull-ups; "
                 "format_if_mount_failed is OFF (will not wipe 16 GB card)",
                 esp_err_to_name(mount_err));
        return;
    }

    sd_info_t info = {};
    if (sd.getInfo(info) == ESP_OK) {
        const double mib = static_cast<double>(info.capacity_bytes) /
                           static_cast<double>(kBytesPerMiB);
        const double gib = static_cast<double>(info.capacity_bytes) /
                           static_cast<double>(kBytesPerGiB);
        ESP_LOGI(TAG,
                 "Card name='%s' bytes=%llu (%.1f MiB / %.2f GiB) "
                 "sector=%lu count=%lu",
                 info.name,
                 (unsigned long long)info.capacity_bytes,
                 mib,
                 gib,
                 (unsigned long)info.sector_size,
                 (unsigned long)info.sector_count);
    } else {
        ESP_LOGW(TAG, "getInfo failed");
    }

    char path[48] = {};
    buildAbsPath(sd, kTestRelPath, path, sizeof(path));

    if (posixWriteTest(path) != ESP_OK) {
        return;
    }
    if (posixReadTest(path) != ESP_OK) {
        return;
    }
    if (posixListRoot(sd) != ESP_OK) {
        return;
    }

    ESP_LOGI(TAG, "All checks passed — entering heartbeat");
    heartbeatLoop(path);
}
