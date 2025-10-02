# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(tempsense C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(tempsense tempsense.cpp ssd1306.c)

pico_set_program_name(tempsense "tempsense")
pico_set_program_version(tempsense "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(tempsense 0)
pico_enable_stdio_usb(tempsense 1)
target_compile_definitions(tempsense PRIVATE LWIP_MQTT=1 PICO_CYW43_ARCH_THREADSAFE_BACKGROUND=1)


# Add the standard library to the build
target_sources(tempsense PRIVATE ${PICO_SDK_PATH}/lib/lwip/src/apps/mqtt/mqtt.c )
# Add the standard include files to the build
target_include_directories(tempsense PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}   # this makes lwipopts.h in your project override the SDK one
    ${CMAKE_CURRENT_LIST_DIR}/pico-devices


)


add_subdirectory("pico-devices")

# Add any user requested libraries
target_link_libraries(tempsense 
        pico_stdlib
        hardware_i2c
        pico_cyw43_arch_lwip_threadsafe_background
        sensor_bme280
        callbacks_blocking
        )

pico_add_extra_outputs(tempsense)

