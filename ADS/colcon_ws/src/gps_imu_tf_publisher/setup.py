from setuptools import find_packages, setup

package_name = 'gps_imu_tf_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yun',
    maintainer_email='dbswkd76@naver.com',
    description='GPS & IMU TF Publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={  # ✅ 실행 가능한 ROS 2 노드 등록
        'console_scripts': [
            'gps_transform = gps_imu_tf_publisher.gps_transform:main',
            'tf_publisher = gps_imu_tf_publisher.tf_publisher:main',
            'imu_to_odom = gps_imu_tf_publisher.imu_to_odom:main',
        ],
    },
)
