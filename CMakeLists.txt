# Jurababa Micromouse - Root CMake Configuration

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
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

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Jurababa C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Source files organized by layer
set(DRIVER_SOURCES
    firmware/drivers/motor.cpp
    firmware/drivers/encoder.cpp
    firmware/drivers/imu.cpp
    firmware/drivers/tof.cpp
    firmware/drivers/battery.cpp
)

set(CONTROL_SOURCES
    firmware/control/pid.cpp
    firmware/control/profile.cpp
    firmware/control/drivetrain.cpp
    firmware/control/robot.cpp
)

set(MAZE_SOURCES
    firmware/maze/maze.cpp
    firmware/maze/mouse.cpp
)

set(NAVIGATION_SOURCES
    firmware/navigation/a_star.cpp
    firmware/navigation/flood_fill.cpp
    firmware/navigation/path_converter.cpp
    firmware/navigation/path_utils.cpp
    firmware/navigation/diagonalizer.cpp
)

set(APP_SOURCES
    firmware/app/bluetooth.cpp
    firmware/app/api.cpp
)

set(MOTOR_LAB_SOURCES
    firmware/motor_lab/motor_lab.cpp
    firmware/motor_lab/profile.cpp
    firmware/motor_lab/reporter.cpp
    firmware/motor_lab/settings.cpp
)

set(COMMON_SOURCES
    firmware/common/log.cpp
)

# Add executable with all sources
add_executable(Jurababa
    ${DRIVER_SOURCES}
    ${CONTROL_SOURCES}
    ${MAZE_SOURCES}
    ${NAVIGATION_SOURCES}
    ${APP_SOURCES}
    ${MOTOR_LAB_SOURCES}
    ${COMMON_SOURCES}
    firmware/main.cpp
)

# Define PICO_BUILD for conditional compilation
target_compile_definitions(Jurababa PRIVATE PICO_BUILD)

pico_set_program_name(Jurababa "Jurababa")
pico_set_program_version(Jurababa "1.0")

# Configure UART stdio pins for HM-10 Bluetooth (GP16=TX, GP17=RX)
target_compile_definitions(Jurababa PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
)

# USB only - UART will be handled manually in MotorLab at 9600 baud
pico_enable_stdio_uart(Jurababa 0)
pico_enable_stdio_usb(Jurababa 1)

# Add the ToF library to the build
add_subdirectory(third_party/vl53l0x_api_rp2040)

# Link libraries
target_link_libraries(Jurababa
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_pwm
    hardware_irq
    hardware_uart
    hardware_i2c
    hardware_adc
    hardware_spi
    vl53l0x_api_rp2040
    pico_multicore
)

# Include directories
target_include_directories(Jurababa PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/firmware
    ${CMAKE_CURRENT_LIST_DIR}/third_party
)

pico_add_extra_outputs(Jurababa)
