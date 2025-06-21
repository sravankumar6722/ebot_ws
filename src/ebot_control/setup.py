import os
from glob import glob
from setuptools import setup

package_name = 'ebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Include all URDF/XACRO files
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # ... rest of your setup configuration
    maintainer='sravan',
    maintainer_email='sravanvintillx@gmail.com',
    description='ROS2 package for ebot control and navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = ebot_control.controller_node:main',
            # Add other executables here
        ],
    },
)