from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rescue_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arepo',
    maintainer_email='nabetse069@gmail.com',
    description='RoboCorea autonomy PoC: TF/EKF/SLAM/Nav2 + Gazebo sim + waypoint runner.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_runner = rescue_nav.waypoint_runner:main',
            'adaptive_odom_covariance = rescue_nav.adaptive_odom_covariance:main',
            'nav_preflight = rescue_nav.nav_preflight:main',
        ],
    },
)
