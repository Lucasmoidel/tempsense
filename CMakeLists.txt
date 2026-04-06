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
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(tempsense C CXX ASM)

pico_sdk_init()

add_executable(tempsense
    tempsense.cpp
    ssd1306.c
)

pico_set_program_name(tempsense "tempsense")
pico_set_program_version(tempsense "0.1")

# USB Logging
pico_enable_stdio_uart(tempsense 0)
pico_enable_stdio_usb(tempsense 1)

# Enable LWIP MQTT only
target_compile_definitions(tempsense PRIVATE LWIP_MQTT=1)

# Add MQTT source file
target_sources(tempsense PRIVATE
    ${PICO_SDK_PATH}/lib/lwip/src/apps/mqtt/mqtt.c
)

target_include_directories(tempsense PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/pico-devices
)

# Do NOT manually set this — the link library below controls the mode
# set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND 1)  <-- REMOVE THIS!

add_subdirectory("pico-devices")

target_link_libraries(tempsense
    pico_stdlib
    hardware_i2c
    pico_cyw43_arch_lwip_threadsafe_background  # This is all you need for background WiFi
    sensor_bme280
    callbacks_blocking
)

pico_add_extra_outputs(tempsense)
