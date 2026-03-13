from setuptools import setup

package_name = 'galaxsea'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
        (
            'share/' + package_name + '/params',
            ['params/ekf.yaml', 'params/navsat.yaml', 'params/slam_params.yaml'],
        ),
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
        ],
    },
)