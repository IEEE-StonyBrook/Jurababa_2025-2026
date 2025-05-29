# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
# set(PICO_SDK_PATH "/Users/elvitigalalis/pico-sdk")
include(pico_sdk_import.cmake)

project(Ratawolfeux C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# Add all cpp files here
add_executable(Ratawolfeux
    Robot/Drivetrain.cpp
    Robot/Motor.cpp
    Robot/Encoder.cpp
    Robot/IMU.cpp
    Robot/ToF.cpp
    Utils/PIDController.cpp
    Utils/LogSystem.cpp
    Virtual/InternalMouse.cpp
    Virtual/MazeGraph.cpp
    Virtual/MazeNode.cpp
    Solver/AStarSolver.cpp
    Main.cpp
)

pico_set_program_name(Ratawolfeux "Ratawolfeux")
pico_set_program_version(Ratawolfeux "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Ratawolfeux 0)
pico_enable_stdio_usb(Ratawolfeux 1)

# Add the ToF library to the build
add_subdirectory(vl53l0x_api_rp2040)

# Add the standard library to the build
target_link_libraries(Ratawolfeux
        pico_stdlib
        hardware_gpio
        hardware_pio
        hardware_pwm
        hardware_irq
        hardware_uart
        hardware_i2c
        vl53l0x_api_rp2040
        )

# Add the standard include files to the build
target_include_directories(Ratawolfeux PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(Ratawolfeux)