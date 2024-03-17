cmake_minimum_required(VERSION 3.5)
project(anymal_c_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
${PROJECT_NAME}/joint_calibrator_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()