cmake_minimum_required(VERSION 3.12)
project(PING_MESSAGES)

# Check if python is installed with the necessary components
find_package(Python3 QUIET REQUIRED COMPONENTS Interpreter)

add_custom_target(generate-messages
  # generate ping messages
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../generate/generate-message.py --output-dir ${CMAKE_CURRENT_SOURCE_DIR}/../message
  COMMENT "Generates ping messages"
)

add_library(PING_MESSAGES INTERFACE)

set(PING_CPP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${PING_CPP_INCLUDE_DIR})
target_include_directories(PING_MESSAGES INTERFACE ${PING_CPP_INCLUDE_DIR})
target_link_libraries(PING_MESSAGES)

add_dependencies(PING_MESSAGES generate-messages)
