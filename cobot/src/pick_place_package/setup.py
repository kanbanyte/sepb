import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pick_place_package'

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
	maintainer='cobot',
	maintainer_email='kanbanyte@outlook.com.au',
	description='TODO: Package description',
	license='TODO: License declaration',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'cobot_node = pick_place_package.cobot_node:main',
			'camera_node = pick_place_package.camera_node:main',
			'gripper_node = pick_place_package.gripper_node:main',
			'main_node = pick_place_package.main_node:main'
		],
	},
)
