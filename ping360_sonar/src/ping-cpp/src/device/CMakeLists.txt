cmake_minimum_required(VERSION 3.12)
project(DEVICE)

# Generate ping-cpp devices
add_custom_target(generate-devices
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../generate/generate-device.py --output-dir ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generates ping device class."
BYPRODUCTS
    ${CMAKE_CURRENT_SOURCE_DIR}/ping-device-ping1d.cpp
)

add_library(
    DEVICE
STATIC
    ping-device-ping1d.cpp
    ping-device.cpp
)

include_directories(${HAL_SOURCE_DIR})

target_link_libraries(
    DEVICE
PRIVATE
    HAL
    PING_MESSAGES
)

target_include_directories(
    DEVICE
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_dependencies(DEVICE generate-devices)
