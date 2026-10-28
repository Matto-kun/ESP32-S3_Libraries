# Main application component
file(GLOB_RECURSE app_sources "${CMAKE_SOURCE_DIR}/src/*.cpp" "${CMAKE_SOURCE_DIR}/src/*.c")

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS
        "${CMAKE_SOURCE_DIR}/include"
    REQUIRES
        st7789
        driver
        freertos
        log
        fatfs
        sdmmc
)
