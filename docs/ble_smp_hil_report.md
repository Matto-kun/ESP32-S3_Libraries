# BLE SMP HIL report (pairing / bonding / encryption)

Date: 2026-10-29  
Board: ESP32-S3 (MAC `e4:b0:63:b3:f6:fa`)  
Passkey profile: SC + MITM + bond, DisplayOnly, static passkey **123456**  
Docs used: Espressif [GATT Security Server walkthrough](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_security_server/tutorial/Gatt_Security_Server_Example_Walkthrough.md), [NimBLE Security](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/ble_get_started/nimble/NimBLE_Security/README.md), [BLE security concepts](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/advanced-development/ble-application-note/connection.html#ble-security)

## Results

| Stack | Env | ADV name | SMP log | Host Bleak | Notes |
|-------|-----|----------|---------|------------|-------|
| Bluedroid | `esp32-s3-bluedroid-sec-peri` | `ESP32-S3-BD-SEC` | `SMP enabled auth=2 iocap=1 passkey=123456`, `SMP:1` | Seen rssi≈−38, service UUID present | UUID in ADV, name in SCAN_RSP |
| NimBLE | `esp32-s3-ble-sec-peri` | `NB-SEC` | `NimBLE SMP enable auth=2 iocap=1 passkey=123456`, `SMP:1` | Name + service UUID in ADV | Short name keeps Flags+name+UUID in 31 B |

## Serial evidence (Bluedroid)

```
BLE_INIT: Feature Config, ... SMP:1, CONNECT:1
bluedroid_api: SMP enabled auth=2 iocap=1 passkey=123456
bluedroid_api: advertising as "ESP32-S3-BD-SEC"
BD_SEC_HIL: Advertising as "ESP32-S3-BD-SEC" bonds=0
```

## Serial evidence (NimBLE)

```
BLE_INIT: Feature Config, ... SMP:1, CONNECT:1
ble_api: NimBLE SMP enable auth=2 iocap=1 passkey=123456
ble_api: primary ADV has name+UUID (no SCAN_RSP)
ble_api: advertising as "NB-SEC"
NB_SEC_HIL: Advertising as "NB-SEC" bonds=0
```

## Phone / bond steps (manual)

1. Open nRF Connect or LightBlue.
2. Connect to `ESP32-S3-BD-SEC` or `NB-SEC`.
3. Enter passkey **123456** when asked.
4. Expect serial `AUTH_CMPL success` (Bluedroid) or `ENC_CHANGE success` / `connection encrypted` (NimBLE).
5. RX writes require encryption when `require_enc_gatt=true`.
6. Reboot: `getBondCount()` / HIL log `bonds=` should be > 0 if the phone bonded.

## API enablement

```cpp
cfg.sec.enable = true;
cfg.sec.auth_mode = /* ScMitmBond */;
cfg.sec.io_cap = /* DisplayOnly */;
cfg.sec.static_passkey = 123456;
cfg.sec.auto_encrypt_on_connect = true;
cfg.sec.require_enc_gatt = true;
```

Default `sec.enable = false` keeps open-GATT apps unchanged.
