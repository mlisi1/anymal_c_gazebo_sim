cmake_minimum_required(VERSION 3.5)
project(anymal_c_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()