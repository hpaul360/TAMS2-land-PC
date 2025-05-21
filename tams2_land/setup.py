import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tams2_land'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hannibal Paul',
    maintainer_email='ubuntu@todo.todo',
    description='A ROS 2 package to process and land for a TAMS2 UAV',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam = tams2_land.landing_processor:main',
            'arm = tams2_land.arms_controller:main',
            'offboard = tams2_land.offboard:main',
        ],
    },
)
