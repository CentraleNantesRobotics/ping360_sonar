# This CMake file allows compiling ping-cpp from submodule

cmake_minimum_required(VERSION 3.12)
project(ping-cpp)

find_package(Boost 1.67 COMPONENTS atomic program_options system thread REQUIRED)
find_package(Threads)

add_library(ping-cpp SHARED
    ping-cpp/src/hal/link/desktop/abstract-link.cpp
    ping-cpp/src/hal/link/desktop/serial-link.cpp
    ping-cpp/src/hal/link/desktop/udp-link.cpp
    ping-cpp/src/hal/time/desktop/ping-time.cpp
    ping-cpp/src/device/ping-device-ping360.cpp
    ping-cpp/src/device/ping-device.cpp
)

target_include_directories(ping-cpp PUBLIC
    ${Boost_INCLUDE_DIR}   
    ping-cpp/src/message ping-cpp/src/hal
)

target_link_libraries(ping-cpp LINK_PUBLIC
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_compile_options(ping-cpp PRIVATE "-Wno-unused-variable -Wno-unused-function -Wno-error")
