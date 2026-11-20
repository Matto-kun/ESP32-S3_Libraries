# Main application / library component sources.
#
# BLUEDROID_PERI_BRINGUP=1 builds only the Bluedroid peripheral HIL image.
# Default build excludes *_bringup.cpp and *_example.cpp (extra app_main).

# Set via PlatformIO: board_build.cmake_extra_args = -DBLUEDROID_PERI_BRINGUP=1
# HCI VHCI: board_build.cmake_extra_args = -DHCI_VHCI_ADV_BRINGUP=1
if(IDF_BLE_COMPAT_BRINGUP OR "$ENV{IDF_BLE_COMPAT_BRINGUP}" STREQUAL "1")
    # Espressif ble_compatibility_test (ADV name BLE_COMP_TEST).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/idf_ble_compat/ble_compatibility_test.c"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src/idf_ble_compat"
    )
elseif(IDF_GATT_SEC_CLIENT_BRINGUP OR "$ENV{IDF_GATT_SEC_CLIENT_BRINGUP}" STREQUAL "1")
    # Espressif gatt_security_client (peer name ESP_BLE_SECURITY).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/idf_gatt_sec_client_bringup.c"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(IDF_GATT_CLIENT_BRINGUP OR "$ENV{IDF_GATT_CLIENT_BRINGUP}" STREQUAL "1")
    # Espressif gatt_client (scans for ESP_GATTS_DEMO).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/idf_gatt_client_bringup.c"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(IDF_HID_BRINGUP OR "$ENV{IDF_HID_BRINGUP}" STREQUAL "1")
    # Espressif ble_hid_device_demo
    file(GLOB hid_sources "${CMAKE_SOURCE_DIR}/src/idf_hid/*.c")
    set(app_sources ${hid_sources})
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src/idf_hid"
    )
elseif(IDF_GATT_SERVER_BRINGUP OR "$ENV{IDF_GATT_SERVER_BRINGUP}" STREQUAL "1")
    # Espressif examples/bluetooth/bluedroid/ble/gatt_server (gatts_demo.c).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/idf_gatt_server_bringup.c"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(HCI_VHCI_ADV_BRINGUP OR "$ENV{HCI_VHCI_ADV_BRINGUP}" STREQUAL "1")
    # Espressif controller_vhci_ble_adv example (controller-only, no Bluedroid).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/hci_vhci_adv_bringup.c"
        "${CMAKE_SOURCE_DIR}/third_party/hci_common/bt_hci_common.c"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/third_party/hci_common/include"
    )
elseif(BLUEDROID_SEC_PERI_BRINGUP OR "$ENV{BLUEDROID_SEC_PERI_BRINGUP}" STREQUAL "1")
    # Bluedroid peri with SC+MITM+bond (ESP32-S3-BD-SEC).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/bluedroid_api.cpp"
        "${CMAKE_SOURCE_DIR}/src/bluedroid_sec_peri_bringup.cpp"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(BLE_SEC_PERI_BRINGUP OR "$ENV{BLE_SEC_PERI_BRINGUP}" STREQUAL "1")
    # NimBLE peri with SC+MITM+bond (ESP32-S3-NB-SEC).
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/ble_api.cpp"
        "${CMAKE_SOURCE_DIR}/src/ble_sec_peri_bringup.cpp"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(QMC5883P_EXAMPLE OR "$ENV{QMC5883P_EXAMPLE}" STREQUAL "1")
    # examples/qmc5883p_example.cpp is the only app_main.
    set(app_sources
        "${CMAKE_SOURCE_DIR}/examples/qmc5883p_example.cpp"
        "${CMAKE_SOURCE_DIR}/src/i2c_api.cpp"
        "${CMAKE_SOURCE_DIR}/src/sensors/qmc5883p_api.cpp"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(HMC5883L_EXAMPLE OR "$ENV{HMC5883L_EXAMPLE}" STREQUAL "1")
    # examples/hmc5883l_example.cpp is the only app_main.
    set(app_sources
        "${CMAKE_SOURCE_DIR}/examples/hmc5883l_example.cpp"
        "${CMAKE_SOURCE_DIR}/src/i2c_api.cpp"
        "${CMAKE_SOURCE_DIR}/src/sensors/hmc5883l_api.cpp"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
elseif(BLUEDROID_PERI_BRINGUP OR "$ENV{BLUEDROID_PERI_BRINGUP}" STREQUAL "1")
    set(app_sources
        "${CMAKE_SOURCE_DIR}/src/bluedroid_api.cpp"
        "${CMAKE_SOURCE_DIR}/src/bluedroid_peri_bringup.cpp"
    )
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
else()
    file(GLOB_RECURSE app_sources
        "${CMAKE_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/*.c"
    )
    list(FILTER app_sources EXCLUDE REGEX ".*/bluedroid_peri_bringup\\.cpp$")
    list(FILTER app_sources EXCLUDE REGEX ".*/bluedroid_sec_peri_bringup\\.cpp$")
    list(FILTER app_sources EXCLUDE REGEX ".*/ble_sec_peri_bringup\\.cpp$")
    list(FILTER app_sources EXCLUDE REGEX ".*/hci_vhci_adv_bringup\\.c$")
    list(FILTER app_sources EXCLUDE REGEX ".*/idf_gatt_server_bringup\\.c$")
    list(FILTER app_sources EXCLUDE REGEX ".*/idf_hid/.*")
    list(FILTER app_sources EXCLUDE REGEX ".*/idf_gatt_client_bringup\\.c$")
    list(FILTER app_sources EXCLUDE REGEX ".*/idf_gatt_sec_client_bringup\\.c$")
    list(FILTER app_sources EXCLUDE REGEX ".*/idf_ble_compat/.*")
    list(FILTER app_sources EXCLUDE REGEX ".*_example\\.cpp$")
    # Bluedroid needs CONFIG_BT_BLUEDROID_ENABLED; keep out of default BT-off builds.
    list(FILTER app_sources EXCLUDE REGEX ".*/bluedroid_api\\.cpp$")
    set(app_includes
        "${CMAKE_SOURCE_DIR}/include"
    )
endif()

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS ${app_includes}
    REQUIRES
        st7789
        driver
        freertos
        log
        fatfs
        sdmmc
        bt
        nvs_flash
        esp_timer
)
