cmake_minimum_required(VERSION 3.16)
project(bur_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# D-7: no nav2. The previous bur_autonomy depended on nav2_behavior_tree,
# nav2_util and nav2_common for exactly two things, both replaced here:
#   - nav2_util euclidean_distance -> include/bur_autonomy/geometry.hpp
#   - nav2_common RewrittenYaml    -> plain launch_ros parameter overrides
#                                     (launch/autonomy.launch.py)
# Fail at configure time if a nav2 dependency is ever added back.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" _bur_package_xml)
if(_bur_package_xml MATCHES "<[a-z_]*depend>nav2")
  message(FATAL_ERROR
    "bur_autonomy must not depend on nav2 (DEVELOPMENT.md D-7). "
    "Add the helper to include/bur_autonomy/geometry.hpp instead.")
endif()

find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bur_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ROS-free leaves + registration (depends on BT.CPP only).
add_library(bur_autonomy_leaves src/leaves.cpp)
target_include_directories(bur_autonomy_leaves PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(bur_autonomy_leaves behaviortree_cpp)

add_executable(manager src/manager_node.cpp)
target_link_libraries(manager bur_autonomy_leaves)
ament_target_dependencies(manager rclcpp behaviortree_cpp bur_msgs nav_msgs std_msgs std_srvs tf2)

# Bench only: kinematic stand-in for the vehicle, the EKF and the camera.
add_executable(bench_sim src/bench_sim_node.cpp)
target_include_directories(bench_sim PRIVATE include)
ament_target_dependencies(bench_sim rclcpp bur_msgs nav_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS bur_autonomy_leaves
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS manager bench_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# behavior_trees/reference/ is installed too. That is harmless: it is never loaded.
install(DIRECTORY behavior_trees config launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_include_directories(test_geometry PRIVATE include)

  ament_add_gtest(test_leaves test/test_leaves.cpp)
  target_link_libraries(test_leaves bur_autonomy_leaves)

  # A-3: every top-level behavior_trees/*.xml must load against the real factory.
  ament_add_gtest(test_tree_loading test/test_tree_loading.cpp)
  target_link_libraries(test_tree_loading bur_autonomy_leaves)
  target_compile_definitions(test_tree_loading PRIVATE
    BUR_TREE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")

  ament_add_gtest(test_mission_sim test/test_mission_sim.cpp)
  target_link_libraries(test_mission_sim bur_autonomy_leaves)
  target_compile_definitions(test_mission_sim PRIVATE
    BUR_TREE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(bur_autonomy_leaves)
ament_export_dependencies(behaviortree_cpp)
ament_package()
