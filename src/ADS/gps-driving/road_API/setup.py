from setuptools import setup

package_name = 'road_API'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-dotenv'],
    zip_safe=True,
    maintainer='c103',
    maintainer_email='c103@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_node = road_API.map_node:main',
        ],
    },
)
