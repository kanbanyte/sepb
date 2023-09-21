import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'onrobot_rg_control'

setup(
	name=package_name,
	version='0.0.0',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages', ['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*config.[pxy][yma]*'))),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='TODO',
	maintainer_email='TODO@users.noreply.github.com',
	description='TODO: Package description',
	license='TODO: License declaration',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'baseOnRobotRG = onrobot_rg_control.baseOnRobotRG:main'
		],
	},
)

# cmake_minimum_required(VERSION 3.5)
# project(onrobot_rg_control)

# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(onrobot_rg_modbus_tcp)
# find_package(onrobot_rg_msgs)
# find_package(std_srvs)

# ament_python_install_package(${PROJECT_NAME})

# # Install Python executables
# # build with `colcon build --symlink-install` to symlink these
# install(PROGRAMS
# 	nodes/OnRobotRGSimpleController.py
# 	nodes/OnRobotRGStatusListener.py
# 	nodes/OnRobotRGSimpleControllerServer.py
# 	DESTINATION lib/${PROJECT_NAME}
# )

# install(
# 	DIRECTORY launch
# 	DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
