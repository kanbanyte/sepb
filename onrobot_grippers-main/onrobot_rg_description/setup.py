import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'onrobot_rg_description'

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
			'node = onrobot_rg_description.node:main'
		],
	},
)

# cmake_minimum_required(VERSION 3.5)
# project(onrobot_rg_description)

# find_package(ament_cmake REQUIRED)

# install(
# 	DIRECTORY launch meshes urdf
# 	DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
