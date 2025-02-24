#!/usr/bin/env python3
# real_hardware.launch.py
#
# 실제 하드웨어 로봇을 위한 launch 파일
# 주요 기능:
# 1. 로봇의 URDF 구조 발행
# 2. 센서들의 TF tree 설정
# 3. 실제 센서와 Nav2가 요구하는 프레임 간의 관계 설정

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로 설정
    gps_wpf_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
    
    # URDF 파일 로드
    urdf = os.path.join(gps_wpf_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # 1. URDF 발행 노드
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # 2. GPS 센서 프레임 설정
        # arguments: [x, y, z, roll, pitch, yaw, parent_frame, child_frame]
        # x, y, z: 센서의 위치 (미터 단위)
        # roll, pitch, yaw: 센서의 방향 (라디안 단위)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=[
                '0',     # x: base_link 기준 전/후 위치 (앞쪽이 양수)
                '0',     # y: base_link 기준 좌/우 위치 (왼쪽이 양수)
                '0.1',   # z: base_link 기준 상/하 위치 (위쪽이 양수)
                '0',     # roll: x축 회전 (라디안)
                '0',     # pitch: y축 회전 (라디안)
                '0',     # yaw: z축 회전 (라디안)
                'base_link',  # parent frame: 기준이 되는 프레임
                'gps_link'    # child frame: GPS 센서의 프레임
            ]
        ),

        # 3. IMU 센서 프레임 설정
        # 위와 동일한 형식으로 설정
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=[
                '0',     # x 위치
                '0',     # y 위치
                '0',     # z 위치
                '0',     # roll
                '0',     # pitch
                '0',     # yaw
                'base_link',  # parent frame
                'imu_link'    # child frame: IMU 센서의 프레임
            ]
        ),

        # 4. LiDAR 센서 프레임 설정 (필요한 경우)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '0',     # x 위치
                '0',     # y 위치
                '0.2',   # z 위치
                '0',     # roll
                '0',     # pitch
                '0',     # yaw
                'base_link',  # parent frame
                'laser_link'  # child frame: LiDAR 센서의 프레임
            ]
        )
    ])