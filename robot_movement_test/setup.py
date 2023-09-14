import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_movement_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resources/' + package_name]),
        ('share/' + package_name, ['package.xml']),

		# Include all launch files.
		(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*config.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cobot',
    maintainer_email='75763222+finnmcgearey@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'test_scaled_joint_trajectory_controller = robot_movement_test.publisher_position_trajectory_controller.py:main'
        ],
    },
)
