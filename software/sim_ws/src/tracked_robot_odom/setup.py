from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tracked_robot_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerardo Escobar',
    maintainer_email='gerardodelcid16@gmail.com',
    description='Tracked-robot planar odometry node (Stage A)',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'tracked_odom_node = tracked_robot_odom.tracked_odom_node:main',
            'scan_sanitizer_node = tracked_robot_odom.scan_sanitizer_node:main',
        ],
    },
)
