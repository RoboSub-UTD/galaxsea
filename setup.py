from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'galaxsea'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='...',
    license='...',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boat_controller = galaxsea.boat_controller:main',
            'lidar_preprocess = galaxsea.lidar_preprocess:main',
            'obstacle_type_map_node = galaxsea.obstacle_type_map:main',
            'image_processing = galaxsea.image_processing:main',
            'auto_thruster_move = galaxsea.auto_thruster_move:main',
        ],
    },
)