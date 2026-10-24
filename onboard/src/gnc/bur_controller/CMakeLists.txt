cmake_minimum_required(VERSION 3.16)
project(bur_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(bur_msgs REQUIRED)

# NOTE: bur-onboard had `include_directories(REQUIRED include/bur_controller)`.
# `REQUIRED` is not a valid argument here, so CMake treated it as a relative
# include path that does not exist. Silent no-op; removed.
include_directories(include/bur_controller)

add_executable(${PROJECT_NAME}
  src/controller_node.cpp
  src/utility_func.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rcl_interfaces geometry_msgs nav_msgs tf2 bur_msgs)

target_include_directories(${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
