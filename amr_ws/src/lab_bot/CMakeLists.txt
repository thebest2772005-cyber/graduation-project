cmake_minimum_required(VERSION 3.8)
project(lab_bot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY
    launch
    config
    maps
    urdf
    my_worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_receiver.py
    scripts/cmd_vel_sender.py
    scripts/scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()