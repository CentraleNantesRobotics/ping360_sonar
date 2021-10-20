cmake_minimum_required(VERSION 3.12)
project(ping-cpp)

find_package(Boost 1.67 COMPONENTS atomic program_options system thread REQUIRED)
find_package(Threads)

add_library(ping-cpp SHARED
    hal/link/desktop/abstract-link.cpp
    hal/link/desktop/serial-link.cpp
    hal/link/desktop/udp-link.cpp
    hal/time/desktop/ping-time.cpp
    device/ping-device-ping1d.cpp
    device/ping-device.cpp
)

target_include_directories(ping-cpp PUBLIC
    ${Boost_INCLUDE_DIR}   
    message hal
)

target_link_libraries(ping-cpp LINK_PUBLIC
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
