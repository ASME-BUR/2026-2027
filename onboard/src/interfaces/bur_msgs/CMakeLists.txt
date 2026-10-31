cmake_minimum_required(VERSION 3.8)
project(bur_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # carried over from bur-interfaces@main
  "msg/CVDetection.msg"
  "msg/CVDetections.msg"
  # new for the servo-chain vehicle
  "msg/SwimCommand.msg"
  "msg/GaitCommand.msg"
  "msg/ServoChainCommand.msg"
  "msg/ServoChainState.msg"
  #
  # NOTE: ThrusterCommand.msg is deliberately NOT carried over. It described
  # eight normalised thruster levels and has no meaning on this vehicle.
  # Command.msg was carried over and then deleted (DEVELOPMENT.md X-6):
  # nothing consumed it, and it had a sway field the vehicle cannot execute.
  #
  # NOTE: bur-interfaces@main listed only geometry_msgs here, while Command.msg
  # and CVDetections.msg both used std_msgs/Header. It built anyway, because
  # geometry_msgs pulls std_msgs in transitively. Declared properly now.
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # xmllint fetches package_format3.xsd from download.ros.org on every run, so
  # it fails with no network. The workspace is meant to build and test offline
  # (bench, pool deck), so skip it.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
