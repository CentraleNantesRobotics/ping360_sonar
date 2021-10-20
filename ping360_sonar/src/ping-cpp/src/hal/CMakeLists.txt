cmake_minimum_required(VERSION 3.12)
project(HAL)

find_package(Boost 1.67 COMPONENTS atomic program_options system thread REQUIRED)
find_package(Threads)
include_directories(${Boost_INCLUDE_DIR})

add_library(
    HAL
    link/desktop/abstract-link.cpp
    link/desktop/serial-link.cpp
    link/desktop/udp-link.cpp
    time/desktop/ping-time.cpp
)

target_include_directories(
    HAL
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    HAL
LINK_PUBLIC
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
