cmake_minimum_required(VERSION 3.16)
project(drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(src/i2c_bus src/bar30_depth_sensor)

# The i2c-dev shim that replaces pigpiod_if2. See ../../../README.md.
add_library(i2c_bus SHARED src/i2c_bus/i2c_bus.cpp src/i2c_bus/i2c_bus.h)
target_include_directories(i2c_bus
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/i2c_bus>
         $<INSTALL_INTERFACE:include/i2c_bus>)

# Carried over from bur-onboard with only its #include changed.
add_library(bar30_depth_sensor SHARED
  src/bar30_depth_sensor/MS5837.cpp
  src/bar30_depth_sensor/MS5837.h)
target_include_directories(bar30_depth_sensor
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bar30_depth_sensor>
         $<INSTALL_INTERFACE:include/bar30_depth_sensor>)
target_link_libraries(bar30_depth_sensor i2c_bus)

# NOTE: bur-onboard gave each library its own export set. That only works while
# nothing links between them; bar30_depth_sensor now depends on i2c_bus, and
# CMake refuses to export a target whose dependency lives in a different set.
# One export set for the whole package.
ament_export_targets(driversTargets HAS_LIBRARY_TARGET)

# NOTE: the install include paths matter and are easy to get wrong.
# `install(DIRECTORY src/ DESTINATION include)` lands the headers at
# include/<subdir>/<header>, so each target's INSTALL_INTERFACE has to name its
# own subdirectory or a downstream `#include "MS5837.h"` will not resolve.
install(DIRECTORY src/ DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

install(TARGETS i2c_bus bar30_depth_sensor
  EXPORT driversTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_package()
