from setuptools import setup
from glob import glob
import os

package_name = 'hardcarry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob(os.path.join('launch','*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c103',
    maintainer_email='c103@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = hardcarry.main_node:main',
            'following_node = hardcarry.following_node:main',
            'joycon_node = hardcarry.joycon_node:main',
            'rp_com_node = hardcarry.rp_com_node:main',
        ],
    },
)
