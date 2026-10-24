cmake_minimum_required(VERSION 3.16)
project(bur_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# TODO(port): no node is built yet. See DEVELOPMENT.md task A-1 - it has to be
# rewritten against bur_msgs/SwimCommand before it is worth compiling, and
# porting it as-is would bake in the holonomic assumption.
#
# When you do write it, you will need:
#   find_package(rclcpp REQUIRED)
#   find_package(behaviortree_cpp REQUIRED)
#   find_package(bur_msgs REQUIRED)
#   find_package(nav_msgs REQUIRED)
# and NOT nav2_behavior_tree - see DEVELOPMENT.md task D-7.

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/)

ament_package()
