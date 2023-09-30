cmake_minimum_required(VERSION 3.5)
project(onrobot_rg_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(onrobot_rg_modbus_tcp)
find_package(onrobot_rg_msgs)
find_package(std_srvs)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# build with `colcon build --symlink-install` to symlink these
install(PROGRAMS
	nodes/rg2_controller.py
	nodes/rg2_listener.py
	nodes/rg2_tcp.py
	nodes/rg2_server.py
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
