import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'onrobot_rg_msgs'

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
			'node = onrobot_rg_msgs.node:main'
		],
	},
)

# cmake_minimum_required(VERSION 3.5)
# project(onrobot_rg_msgs)

# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)

# set(msg_files
# 	"msg/OnRobotRGInput.msg"
# 	"msg/OnRobotRGOutput.msg"
# 	"srv/SetCommand.srv"
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
# 	${msg_files}
# 	DEPENDENCIES std_msgs
# )

# ament_export_dependencies(rosidl_default_runtime)

# ament_package()
