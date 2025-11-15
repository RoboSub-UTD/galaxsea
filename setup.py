from setuptools import setup

package_name = 'galaxsea26'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'boat_controller = galaxsea26.boat_controller:main',
            'lidar_preprocess = galaxsea26.lidar_preprocess:main',
            'image_processing = galaxsea26.image_processing:main',
        ],
    },
)