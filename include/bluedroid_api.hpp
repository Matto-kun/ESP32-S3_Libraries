/**
 * @file        bluedroid_api.hpp
 * @brief       Bluedroid BLE GATT peripheral and central drivers for ESP32-S3
 * @author      Mat RB
 * @date        2026-10-28
 * @version     1.1.0
 *
 * @details
 *   Provides BluedroidPeripheral (GATT server) and BluedroidCentral (GATT client)
 *   on the ESP-IDF Bluedroid host. Device name and 128-bit UUIDs come from
 *   bluedroid_cfg_t. Bluedroid / controller types stay in bluedroid_api.cpp.
 *   Only one role may be active after init().
 *   Optional LE Secure Connections pairing/bonding via bluedroid_sec_cfg_t
 *   (default disabled so open-GATT apps keep working).
 *
 * @note ESP32-S3 supports BLE only (no classic BR/EDR). This API releases
 *       classic controller memory at init. Callbacks run in the Bluedroid
 *       BTU / GATT context. Keep them short. Do not block, and do not allocate
 *       from the heap inside them.
 *
 * @note Do not enable NimBLE (ble_api) in the same firmware image. The two
 *       hosts cannot share the controller.
 *
 * @platform    ESP32-S3 (ESP-IDF via PlatformIO, Bluedroid)
 */

#ifndef BLUEDROID_API_HPP
#define BLUEDROID_API_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

/** Default advertise / local name for BluedroidPeripheral. */
static constexpr const char* kBluedroidDefaultDeviceName = "ESP32-S3-BLE";

/** Default central scan duration; 0 means scan until stopScan(). */
static constexpr uint32_t kBluedroidDefaultScanMs = 10000U;

/** Max payload for a single notify or write in this API. */
static constexpr size_t kBluedroidMaxPayloadBytes = 512U;

/** Default static passkey for DisplayOnly MITM (0..999999). */
static constexpr uint32_t kBluedroidDefaultStaticPasskey = 123456U;

/**
 * @brief Bluetooth device address, 6 bytes wire order.
 */
typedef struct {
    uint8_t bytes[6];
} bluedroid_bd_addr_t;

/**
 * @brief LE authentication / bonding mode for bluedroid_sec_cfg_t.
 */
enum class eBluedroidAuthMode : uint8_t {
    None = 0,
    Bond,
    ScMitmBond,
};

/**
 * @brief LE IO capability for pairing.
 */
enum class eBluedroidIoCap : uint8_t {
    None = 0,
    DisplayOnly,
    DisplayYesNo,
    KeyboardOnly,
    KeyboardDisplay,
};

/**
 * @brief Optional Security Manager configuration.
 *
 * @note enable defaults to false. Secure HIL uses ScMitmBond + DisplayOnly
 *       + static_passkey 123456 (Espressif ble_compatibility_test profile).
 */
typedef struct {
    bool               enable                  = false;
    eBluedroidAuthMode auth_mode               = eBluedroidAuthMode::ScMitmBond;
    eBluedroidIoCap    io_cap                  = eBluedroidIoCap::DisplayOnly;
    uint32_t           static_passkey          = kBluedroidDefaultStaticPasskey;
    bool               auto_encrypt_on_connect = true;
    bool               require_enc_gatt        = true;
} bluedroid_sec_cfg_t;

/**
 * @brief 128-bit UUID as 16 bytes in Bluetooth wire order
 *        (little-endian of the canonical UUID string).
 */
typedef struct {
    uint8_t bytes[16];
} bluedroid_uuid_bytes_t;

/**
 * @brief Library default service UUID
 *        (a1b2c3d4-e5f6-7890-abcd-ef1234567890).
 */
static constexpr bluedroid_uuid_bytes_t kBluedroidDefaultServiceUuid = {{
    0x90, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/** Default RX (write) UUID …7891. */
static constexpr bluedroid_uuid_bytes_t kBluedroidDefaultRxUuid = {{
    0x91, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/** Default TX (notify) UUID …7892. */
static constexpr bluedroid_uuid_bytes_t kBluedroidDefaultTxUuid = {{
    0x92, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/** Default status (notify) UUID …7893. */
static constexpr bluedroid_uuid_bytes_t kBluedroidDefaultStatusUuid = {{
    0x93, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab,
    0x90, 0x78, 0xf6, 0xe5, 0xd4, 0xc3, 0xb2, 0xa1
}};

/**
 * @brief Shared Bluedroid BLE configuration for peripheral and central.
 *
 * Override UUID fields for a product profile. Defaults are library-owned.
 */
typedef struct {
    const char*              device_name      = kBluedroidDefaultDeviceName;
    bluedroid_uuid_bytes_t   service_uuid     = kBluedroidDefaultServiceUuid;
    bluedroid_uuid_bytes_t   rx_uuid          = kBluedroidDefaultRxUuid;
    bluedroid_uuid_bytes_t   tx_uuid          = kBluedroidDefaultTxUuid;
    bluedroid_uuid_bytes_t   status_uuid      = kBluedroidDefaultStatusUuid;
    bool                     auto_advertise   = true;
    uint32_t                 scan_ms          = kBluedroidDefaultScanMs;
    const char*              peer_name_filter = nullptr;
    bluedroid_sec_cfg_t      sec              = {};
} bluedroid_cfg_t;

/** Connection state callback (Bluedroid BTU / GATT context). */
typedef void (*bluedroid_conn_cb_t)(bool connected, uint16_t conn_id);

/** RX / notify payload callback (Bluedroid BTU / GATT context). */
typedef void (*bluedroid_data_cb_t)(const uint8_t* data, size_t len);

/**
 * @brief Pairing / encryption complete (Bluedroid BTU context).
 * @param ok true if AUTH_CMPL succeeded
 * @param fail_reason stack fail reason when ok is false
 */
typedef void (*bluedroid_auth_cb_t)(bool ok, uint8_t fail_reason);

/**
 * @brief Passkey to show on a display (DisplayOnly).
 */
typedef void (*bluedroid_passkey_cb_t)(uint32_t passkey);

/**
 * @brief GATT server / peripheral role (Bluedroid).
 *
 * Primary ADV carries Flags + service_uuid. SCAN_RSP carries device_name
 * (31-byte ADV limit). Exposes RX (write), TX (notify), and status (notify).
 * Hardware is untouched until init().
 */
class BluedroidPeripheral
{
public:
    explicit BluedroidPeripheral(const bluedroid_cfg_t& config = bluedroid_cfg_t{});
    ~BluedroidPeripheral();

    BluedroidPeripheral(const BluedroidPeripheral&)            = delete;
    BluedroidPeripheral& operator=(const BluedroidPeripheral&) = delete;

    /**
     * @brief Start controller, Bluedroid host, and GATT server.
     * @return ESP_OK on success, or an ESP-IDF error code.
     */
    esp_err_t init();

    /**
     * @brief Stop advertising and tear down Bluedroid / controller.
     * @return ESP_OK on success, or an ESP-IDF error code.
     */
    esp_err_t deinit();

    esp_err_t startAdvertising();
    esp_err_t stopAdvertising();

    esp_err_t notifyTx(const uint8_t* data, size_t len);
    esp_err_t notifyStatus(const uint8_t* data, size_t len);
    esp_err_t notifyStatus(const char* utf8);

    void setConnCallback(bluedroid_conn_cb_t cb);
    void setRxCallback(bluedroid_data_cb_t cb);
    void setAuthCallback(bluedroid_auth_cb_t cb);
    void setPasskeyDisplayCallback(bluedroid_passkey_cb_t cb);

    /**
     * @brief Start LE encryption / pairing with the connected peer.
     */
    esp_err_t requestEncryption();

    int       getBondCount() const;
    esp_err_t removeBond(const bluedroid_bd_addr_t& addr);
    esp_err_t clearBonds();

    bool                     isConnected() const;
    bool                     isEncrypted() const;
    bool                     isBonded() const;
    bool                     isTxSubscribed() const;
    bool                     isStatusSubscribed() const;
    bool                     isInited() const;
    const bluedroid_cfg_t&   getConfig() const;

private:
    bluedroid_cfg_t         m_config = {};
    bool                    m_inited = false;
    bluedroid_conn_cb_t     m_conn_cb = nullptr;
    bluedroid_data_cb_t     m_rx_cb   = nullptr;
    bluedroid_auth_cb_t     m_auth_cb = nullptr;
    bluedroid_passkey_cb_t  m_passkey_cb = nullptr;
};

/**
 * @brief GATT client / central role (Bluedroid).
 *
 * Scans for a peer that advertises service_uuid (and optional name filter),
 * connects, discovers RX/TX/status, and can write RX and subscribe to notifies.
 */
class BluedroidCentral
{
public:
    explicit BluedroidCentral(const bluedroid_cfg_t& config = bluedroid_cfg_t{});
    ~BluedroidCentral();

    BluedroidCentral(const BluedroidCentral&)            = delete;
    BluedroidCentral& operator=(const BluedroidCentral&) = delete;

    esp_err_t init();
    esp_err_t deinit();

    esp_err_t startScan();
    esp_err_t stopScan();

    esp_err_t disconnect();

    esp_err_t writeRx(const uint8_t* data, size_t len);
    esp_err_t writeRx(const char* utf8);

    esp_err_t subscribeTx(bool enable);
    esp_err_t subscribeStatus(bool enable);

    void setConnCallback(bluedroid_conn_cb_t cb);
    void setTxNotifyCallback(bluedroid_data_cb_t cb);
    void setStatusNotifyCallback(bluedroid_data_cb_t cb);
    void setAuthCallback(bluedroid_auth_cb_t cb);
    void setPasskeyDisplayCallback(bluedroid_passkey_cb_t cb);

    esp_err_t requestEncryption();

    int       getBondCount() const;
    esp_err_t removeBond(const bluedroid_bd_addr_t& addr);
    esp_err_t clearBonds();

    bool                     isConnected() const;
    bool                     isEncrypted() const;
    bool                     isBonded() const;
    bool                     isInited() const;
    bool                     isReady() const;
    const bluedroid_cfg_t&   getConfig() const;

private:
    bluedroid_cfg_t         m_config = {};
    bool                    m_inited = false;
    bluedroid_conn_cb_t     m_conn_cb   = nullptr;
    bluedroid_data_cb_t     m_tx_cb     = nullptr;
    bluedroid_data_cb_t     m_status_cb = nullptr;
    bluedroid_auth_cb_t     m_auth_cb   = nullptr;
    bluedroid_passkey_cb_t  m_passkey_cb = nullptr;
};

#endif /* BLUEDROID_API_HPP */
