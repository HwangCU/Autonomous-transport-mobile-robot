#!/usr/bin/env python3
# gps_waypoint_follower.launch.py
#
# 이 파일은 Nav2 GPS Waypoint Following을 위한 메인 launch 파일입니다.
# 주요 기능:
# 1. 실제 하드웨어 설정 실행 (real_hardware.launch.py)
# 2. robot_localization 실행 (GPS + IMU fusion)
# 3. Nav2 네비게이션 스택 실행
# 4. 시각화 도구 실행 (선택적: RVIZ, Mapviz)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 필요한 패키지 디렉토리 경로 설정
    bringup_dir = get_package_share_directory('nav2_bringup')
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower_demo")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    params_dir = os.path.join(gps_wpf_dir, "config")

    # Nav2 파라미터 파일 로드
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # 시각화 도구 사용 여부 설정
    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')
    # Launch 파일 인자 설정
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')
    
    # 수정된 부분: gazebo_cmd를 real_hardware_cmd로 변경
    # 시뮬레이션 대신 실제 하드웨어 설정 사용
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'real_hardware.launch.py'))
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'dual_ekf_navsat.launch.py'))
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False", # 실제 하드웨어이므로 False로 변경
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz)
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # simulator launch
    ld.add_action(gazebo_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)

    return ld
