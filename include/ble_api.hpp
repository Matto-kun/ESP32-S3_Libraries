/**
 * @file        ble_api.hpp
 * @brief       Generic NimBLE GATT peripheral and central drivers for ESP32-S3
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.0.0
 *
 * @details
 *   Provides BlePeripheral (GATT server) and BleCentral (GATT client).
 *   Device name and 128-bit UUIDs come from ble_cfg_t. NimBLE types stay
 *   in ble_api.cpp. Only one role may be active after init().
 *
 * @note Callbacks run in the NimBLE host context. Keep them short.
 *       Do not block, and do not allocate from the heap inside them.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO, NimBLE)
 */

#ifndef BLE_API_HPP
#define BLE_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

/** Default advertise / local name for BlePeripheral. */
static constexpr const char* kBleDefaultDeviceName = "ESP32-S3-BLE";

/** Default central scan duration; 0 means scan until stopScan(). */
static constexpr uint32_t kBleDefaultScanMs = 10000U;

/** Max payload for a single notify or write in this API. */
static constexpr size_t kBleMaxPayloadBytes = 512U;

/**
 * @note Set connection / data callbacks before init(), or keep them short.
 *       Callbacks run on the NimBLE host task.
 */

/**
 * @brief 128-bit UUID as 16 bytes in NimBLE / Bluetooth wire order
 *        (little-endian of the canonical UUID string).
 *
 * @note Named ble_uuid_bytes_t so it does not clash with NimBLE ble_uuid128_t.
 */
typedef struct {
    uint8_t bytes[16];
} ble_uuid_bytes_t;

/**
 * @brief Library default service UUID
 *        (a1b2c3d4-e5f6-7890-abcd-ef1234567890).
 */
static constexpr ble_uuid_bytes_t kBleDefaultServiceUuid = {{
    0x90, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/** Default RX (write) UUID …7891. */
static constexpr ble_uuid_bytes_t kBleDefaultRxUuid = {{
    0x91, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/** Default TX (notify) UUID …7892. */
static constexpr ble_uuid_bytes_t kBleDefaultTxUuid = {{
    0x92, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/** Default status (notify) UUID …7893. */
static constexpr ble_uuid_bytes_t kBleDefaultStatusUuid = {{
    0x93, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/**
 * @brief Shared BLE configuration for peripheral and central.
 *
 * Override UUID fields for a product profile. Defaults are library-owned.
 */
typedef struct {
    const char*     device_name      = kBleDefaultDeviceName;
    ble_uuid_bytes_t service_uuid    = kBleDefaultServiceUuid;
    ble_uuid_bytes_t rx_uuid         = kBleDefaultRxUuid;
    ble_uuid_bytes_t tx_uuid         = kBleDefaultTxUuid;
    ble_uuid_bytes_t status_uuid     = kBleDefaultStatusUuid;
    bool            auto_advertise   = true;
    uint32_t        scan_ms          = kBleDefaultScanMs;
    const char*     peer_name_filter = nullptr;
} ble_cfg_t;

/** Connection state callback (NimBLE host context). */
typedef void (*ble_conn_cb_t)(bool connected, uint16_t conn_handle);

/** RX / notify payload callback (NimBLE host context). */
typedef void (*ble_data_cb_t)(const uint8_t* data, size_t len);

/**
 * @brief GATT server / peripheral role.
 *
 * Advertises device_name and service_uuid. Exposes RX (write), TX (notify),
 * and status (notify). Hardware is untouched until init().
 */
class BlePeripheral
{
public:
    explicit BlePeripheral(const ble_cfg_t& config = ble_cfg_t{});
    ~BlePeripheral();

    BlePeripheral(const BlePeripheral&)            = delete;
    BlePeripheral& operator=(const BlePeripheral&) = delete;

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t startAdvertising();
    esp_err_t stopAdvertising();

    esp_err_t notifyTx(const uint8_t* data, size_t len);
    esp_err_t notifyStatus(const uint8_t* data, size_t len);
    esp_err_t notifyStatus(const char* utf8);

    void setConnCallback(ble_conn_cb_t cb);
    void setRxCallback(ble_data_cb_t cb);

    bool              isConnected() const;
    bool              isTxSubscribed() const;
    bool              isStatusSubscribed() const;
    bool              isInited() const;
    const ble_cfg_t&  getConfig() const;

private:
    ble_cfg_t     m_config = {};
    bool          m_inited = false;
    ble_conn_cb_t m_conn_cb = nullptr;
    ble_data_cb_t m_rx_cb   = nullptr;
};

/**
 * @brief GATT client / central role.
 *
 * Scans for a peer that advertises service_uuid (and optional name filter),
 * connects, discovers RX/TX/status, and can write RX and subscribe to notifies.
 */
class BleCentral
{
public:
    explicit BleCentral(const ble_cfg_t& config = ble_cfg_t{});
    ~BleCentral();

    BleCentral(const BleCentral&)            = delete;
    BleCentral& operator=(const BleCentral&) = delete;

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t startScan();
    esp_err_t stopScan();

    esp_err_t disconnect();

    esp_err_t writeRx(const uint8_t* data, size_t len);
    esp_err_t writeRx(const char* utf8);

    esp_err_t subscribeTx(bool enable);
    esp_err_t subscribeStatus(bool enable);

    void setConnCallback(ble_conn_cb_t cb);
    void setTxNotifyCallback(ble_data_cb_t cb);
    void setStatusNotifyCallback(ble_data_cb_t cb);

    bool              isConnected() const;
    bool              isInited() const;
    bool              isReady() const;
    const ble_cfg_t&  getConfig() const;

private:
    ble_cfg_t     m_config = {};
    bool          m_inited = false;
    ble_conn_cb_t m_conn_cb    = nullptr;
    ble_data_cb_t m_tx_cb      = nullptr;
    ble_data_cb_t m_status_cb  = nullptr;
};

#endif /* BLE_API_HPP */
