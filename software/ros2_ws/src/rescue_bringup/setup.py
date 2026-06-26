import os
from glob import glob

from setuptools import setup

package_name = 'rescue_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ship the systemd unit templates in the package share so they are
        # version-controlled and can be copied/symlinked into the user systemd
        # dir on each robot (see systemd/README.md).
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gerardo',
    maintainer_email='gerardodelcid16@gmail.com',
    description='Per-host bringup + robot_manager sensor lifecycle service.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_manager = rescue_bringup.robot_manager:main',
            'map_manager = rescue_bringup.map_manager:main',
        ],
    },
)
