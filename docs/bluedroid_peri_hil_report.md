# Bluedroid peripheral HIL report

**Date:** 2026-10-28  
**Env:** `esp32-s3-bluedroid-peri`  
**Port:** `/dev/ttyACM0`  
**MAC:** `e4:b0:63:b3:f6:fa`  
**Host check:** Bleak (Python) on laptop HCI `hci0`

## Results

| Step | Result | Evidence |
|------|--------|----------|
| Build / flash | PASS | App boots, ESP-IDF 6.0.1, Bluedroid controller init |
| GATT service start | PASS | `GATT service started` |
| Advertising | PASS | `advertising as "ESP32-S3-BLE"` |
| Host ADV visibility | PASS | Bleak scan: `E4:B0:63:B3:F6:FA name='ESP32-S3-BLE' rssi=-36` |
| Connect | PASS | `peri connected`, host `Connected: True` |
| ATT MTU | PASS | `ATT MTU conn=0 mtu=517` |
| RX write | PASS | Host wrote `HIL-RX-2`; serial `RX text="HIL-RX-2"` |
| Status notify | PASS | Host received `heartbeat N` after CCCD subscribe |
| Disconnect + auto ADV | PASS | `ADV restart scheduled in 300 ms` then advertising again |

TX characteristic notify was not exercised in bring-up (heartbeat uses status only). That is expected for this HIL image.

## Serial excerpts

```
I (...) BLE_INIT: Bluetooth MAC: e4:b0:63:b3:f6:fa
I (...) bluedroid_api: GATT service started
W (...) bluedroid_api: service UUID not in ADV (name-only fallback)
I (...) bluedroid_api: advertising as "ESP32-S3-BLE"
I (...) BD_PERI_HIL: Advertising as "ESP32-S3-BLE" (Bluedroid)
```

```
I (...) bluedroid_api: peri connected conn_id=0
I (...) BD_PERI_HIL: conn=1 handle=0
I (...) bluedroid_api: ATT MTU conn=0 mtu=517
I (...) bluedroid_api: status CCCD notify=1
I (...) BD_PERI_HIL: RX 8 bytes
I (...) BD_PERI_HIL: RX text="HIL-RX-2"
...
I (...) bluedroid_api: peri disconnect reason=0x13
I (...) bluedroid_api: ADV restart scheduled in 300 ms
I (...) bluedroid_api: advertising as "ESP32-S3-BLE"
```

## GATT UUIDs observed

| Role | UUID |
|------|------|
| Service | `a1b2c3d4-e5f6-7890-abcd-ef1234567890` |
| RX (write) | `a1b2c3d4-e5f6-7890-abcd-ef1234567891` |
| TX (notify) | `a1b2c3d4-e5f6-7890-abcd-ef1234567892` |
| Status (notify) | `a1b2c3d4-e5f6-7890-abcd-ef1234567893` |

## Caveats (vs NimBLE)

1. **ADV layout (fixed after first HIL):** Primary ADV = Flags + 128-bit service UUID. SCAN_RSP = complete local name. Name + UUID cannot share one 31-byte ADV PDU. Name-only ADV hid the device from many phone OS scanners (Espressif: Apple Settings often requires a service UUID in ADV).
2. **Host stack exclusive:** Bluedroid and NimBLE cannot run in the same firmware image. Product apps stay on NimBLE; this env is HIL-only.
3. **`bluetoothctl` on this host:** D-Bus client crashed (segfault). Bleak + BlueZ worked for scan/connect.
4. **Classic BT:** App releases classic controller memory (`ESP_BT_MODE_CLASSIC_BT`). ESP32-S3 is BLE-only.

## Verdict

Peripheral bring-up on the connected board is **PASS** for init, advertise, host visibility, connect, RX write, status notify, and deferred ADV restart.
