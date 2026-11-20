/**
 * @file        ble_api.hpp
 * @brief       Generic NimBLE GATT peripheral and central drivers for ESP32-S3
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.1.0
 *
 * @details
 *   Provides BlePeripheral (GATT server) and BleCentral (GATT client).
 *   Device name and 128-bit UUIDs come from ble_cfg_t. NimBLE types stay
 *   in ble_api.cpp. Only one role may be active after init().
 *   Optional LE Secure Connections pairing/bonding via ble_sec_cfg_t
 *   (default disabled so open-GATT apps keep working).
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

/** Default static passkey for DisplayOnly MITM (0..999999). */
static constexpr uint32_t kBleDefaultStaticPasskey = 123456U;

/**
 * @brief Bluetooth device address (public or random), 6 bytes wire order.
 */
typedef struct {
    uint8_t bytes[6];
} ble_bd_addr_t;

/**
 * @brief LE authentication / bonding mode for ble_sec_cfg_t.
 */
enum class eBleAuthMode : uint8_t {
    None = 0,     /**< No bonding / no MITM (open link). */
    Bond,         /**< Just Works bond (no MITM). */
    ScMitmBond,   /**< LE Secure Connections + MITM + bond. */
};

/**
 * @brief LE IO capability for pairing.
 */
enum class eBleIoCap : uint8_t {
    None = 0,
    DisplayOnly,
    DisplayYesNo,
    KeyboardOnly,
    KeyboardDisplay,
};

/**
 * @brief Optional Security Manager configuration.
 *
 * @note enable defaults to false. When true, HIL / product secure profiles
 *       typically use ScMitmBond + DisplayOnly + static_passkey 123456.
 */
typedef struct {
    bool         enable                 = false;
    eBleAuthMode auth_mode              = eBleAuthMode::ScMitmBond;
    eBleIoCap    io_cap                 = eBleIoCap::DisplayOnly;
    uint32_t     static_passkey         = kBleDefaultStaticPasskey;
    bool         auto_encrypt_on_connect = true;
    bool         require_enc_gatt       = true;
} ble_sec_cfg_t;

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
    const char*      device_name      = kBleDefaultDeviceName;
    ble_uuid_bytes_t service_uuid     = kBleDefaultServiceUuid;
    ble_uuid_bytes_t rx_uuid          = kBleDefaultRxUuid;
    ble_uuid_bytes_t tx_uuid          = kBleDefaultTxUuid;
    ble_uuid_bytes_t status_uuid      = kBleDefaultStatusUuid;
    bool             auto_advertise   = true;
    uint32_t         scan_ms          = kBleDefaultScanMs;
    const char*      peer_name_filter = nullptr;
    ble_sec_cfg_t    sec              = {};
} ble_cfg_t;

/** Connection state callback (NimBLE host context). */
typedef void (*ble_conn_cb_t)(bool connected, uint16_t conn_handle);

/** RX / notify payload callback (NimBLE host context). */
typedef void (*ble_data_cb_t)(const uint8_t* data, size_t len);

/**
 * @brief Pairing / encryption complete (NimBLE host context).
 * @param ok true if encryption / auth succeeded
 * @param fail_reason stack reason when ok is false (0 when ok)
 */
typedef void (*ble_auth_cb_t)(bool ok, uint8_t fail_reason);

/**
 * @brief Passkey to show on a display (DisplayOnly).
 * @param passkey six-digit value 0..999999
 */
typedef void (*ble_passkey_cb_t)(uint32_t passkey);

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
    void setAuthCallback(ble_auth_cb_t cb);
    void setPasskeyDisplayCallback(ble_passkey_cb_t cb);

    /**
     * @brief Start LE encryption / pairing with the connected peer.
     * @return ESP_OK, ESP_ERR_INVALID_STATE, or ESP_ERR_NOT_SUPPORTED
     */
    esp_err_t requestEncryption();

    int       getBondCount() const;
    esp_err_t removeBond(const ble_bd_addr_t& addr);
    esp_err_t clearBonds();

    bool              isConnected() const;
    bool              isEncrypted() const;
    bool              isBonded() const;
    bool              isTxSubscribed() const;
    bool              isStatusSubscribed() const;
    bool              isInited() const;
    const ble_cfg_t&  getConfig() const;

private:
    ble_cfg_t         m_config = {};
    bool              m_inited = false;
    ble_conn_cb_t     m_conn_cb = nullptr;
    ble_data_cb_t     m_rx_cb   = nullptr;
    ble_auth_cb_t     m_auth_cb = nullptr;
    ble_passkey_cb_t  m_passkey_cb = nullptr;
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
    void setAuthCallback(ble_auth_cb_t cb);
    void setPasskeyDisplayCallback(ble_passkey_cb_t cb);

    esp_err_t requestEncryption();

    int       getBondCount() const;
    esp_err_t removeBond(const ble_bd_addr_t& addr);
    esp_err_t clearBonds();

    bool              isConnected() const;
    bool              isEncrypted() const;
    bool              isBonded() const;
    bool              isInited() const;
    bool              isReady() const;
    const ble_cfg_t&  getConfig() const;

private:
    ble_cfg_t         m_config = {};
    bool              m_inited = false;
    ble_conn_cb_t     m_conn_cb    = nullptr;
    ble_data_cb_t     m_tx_cb      = nullptr;
    ble_data_cb_t     m_status_cb  = nullptr;
    ble_auth_cb_t     m_auth_cb    = nullptr;
    ble_passkey_cb_t  m_passkey_cb = nullptr;
};

#endif /* BLE_API_HPP */
