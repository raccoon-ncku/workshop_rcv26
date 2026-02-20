from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ch1_basics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chia-Ching Yen',
    maintainer_email='ccyen@umich.edu',
    description='Chapter 1: ROS 2 Basics and Kinematic Control for RCCN Kuka Cell',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subset_mover = ch1_basics.subset_mover:main',
        ],
    },
)