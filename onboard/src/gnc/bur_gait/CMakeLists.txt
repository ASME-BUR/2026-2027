cmake_minimum_required(VERSION 3.16)
project(bur_gait)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bur_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/gait_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs bur_msgs)

target_include_directories(${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

# TODO(port): add a unit test for bur_gait::jointAngles. It is a pure function
# with no ROS dependency, so it is the one piece of this stack that is cheap to
# test properly, and the phase/envelope maths is exactly where an off-by-one
# will hide.

ament_package()
