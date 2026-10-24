cmake_minimum_required(VERSION 3.16)
project(bur_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY params launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
