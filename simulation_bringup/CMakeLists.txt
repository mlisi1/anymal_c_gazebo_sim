cmake_minimum_required(VERSION 3.5)
project(simulation_bringup)

find_package(ament_cmake REQUIRED)
find_package(rf2o REQUIRED)
find_package(nav_msgs REQUIRED)


install(DIRECTORY launch config
DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS ${PROJECT_NAME}/odom_to_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
