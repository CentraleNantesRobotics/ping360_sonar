cmake_minimum_required(VERSION 3.8)
project(altimeter_ping360)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensors_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

# Include other libraries
target_link_libraries(altimeter_ping360 Eigen3::Eigen)

# Link ROS 2 dependencies -- only links the ament libraries
ament_target_dependencies(altimeter_ping360 ping360_sonar_msgs image_transport sensors_msgs std_msgs rclcpp)

# Install the executable
install(TARGETS altimeter_ping360
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ping360_sonar_msgs)

ament_package()
