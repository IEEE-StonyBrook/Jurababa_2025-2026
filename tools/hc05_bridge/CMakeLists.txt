cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in the Pico SDK from the repo root (re-uses the same import file
# the main firmware uses, so the SDK version is consistent).
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)

project(hc05_bridge C CXX ASM)

pico_sdk_init()

add_executable(hc05_bridge main.cpp)

pico_set_program_name(hc05_bridge "hc05_bridge")
pico_set_program_version(hc05_bridge "1.0")

# USB CDC for the host terminal; UART stdio off (UART0 is dedicated to HC-05).
pico_enable_stdio_usb(hc05_bridge 1)
pico_enable_stdio_uart(hc05_bridge 0)

target_link_libraries(hc05_bridge
    pico_stdlib
    hardware_uart
    hardware_gpio
)

pico_add_extra_outputs(hc05_bridge)
