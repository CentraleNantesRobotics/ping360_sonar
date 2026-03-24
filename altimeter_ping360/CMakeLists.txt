cmake_minimum_required(VERSION 3.8)
project(altimeter_ping360)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)

# Executable
add_executable(altimeter_ping360
  src/altimeter_ping360.cpp
  src/altimeter.cpp
)

# Include directory (so <altimeter_ping360/altimeter.hpp> works)
target_include_directories(altimeter_ping360 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS 2 dependencies
ament_target_dependencies(altimeter_ping360 ping360_sonar_msgs rclcpp)

# Install the executable
install(TARGETS altimeter_ping360
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ping360_sonar_msgs)

ament_package()
