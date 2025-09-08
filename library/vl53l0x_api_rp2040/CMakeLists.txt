
add_library(vl53l0x_api_rp2040 INTERFACE)
target_sources(vl53l0x_api_rp2040 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_calibration.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_core.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_ranging.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api_strings.c
    ${CMAKE_CURRENT_LIST_DIR}/core/src/vl53l0x_api.c 
    ${CMAKE_CURRENT_LIST_DIR}/platform/src/vl53l0x_rp2040.c
    ${CMAKE_CURRENT_LIST_DIR}/platform/src/vl53l0x_platform.c 
)

target_include_directories(vl53l0x_api_rp2040 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/core/inc
    ${CMAKE_CURRENT_LIST_DIR}/core/src
    ${CMAKE_CURRENT_LIST_DIR}/platform/inc
    ${CMAKE_CURRENT_LIST_DIR}/platform/src
)

target_link_libraries(vl53l0x_api_rp2040 INTERFACE
        pico_stdlib
        hardware_i2c
)
