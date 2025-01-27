from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
           glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
           glob('config/*.csv')),
    ],
    install_requires=[
        'setuptools',
        'custom_interfaces',
        ],
    zip_safe=True,
    maintainer='c103',
    maintainer_email='c103@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follower = gps_navigation.waypoint_follower:main',
        ],
    },
)
