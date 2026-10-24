cmake_minimum_required(VERSION 3.16)
project(bur_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(drivers REQUIRED)

include_directories(include/${PROJECT_NAME})

add_executable(bar30_node src/bar30_node.cpp)
ament_target_dependencies(bar30_node rclcpp std_msgs nav_msgs)
target_link_libraries(bar30_node
  drivers::bar30_depth_sensor
  drivers::i2c_bus)

target_include_directories(bar30_node
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")

install(TARGETS bar30_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# TODO(port): the IMU node is intentionally absent. The IMU arrives over
# micro-ROS from the MCU as sensor_msgs/Imu on /imu. If that ever changes,
# bur-onboard's imu_node.cpp (Xsens MTi over I2C) is in git history - but note
# it was commented out of CMakeLists there and had not been built in a year.

ament_package()
