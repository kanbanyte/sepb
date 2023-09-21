cmake_minimum_required(VERSION 3.5)
project(onrobot_rg_modbus_tcp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})


# Install Python executables
# build with `colcon build --symlink-install` to symlink these
#install(PROGRAMS
#  src/onrobot_rg_modbus_tcp/comModbusTcp.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()