import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # GPS 데이터를 활용하여 base_footprint 위치를 업데이트
        launch_ros.actions.Node(
            package='gps_imu_tf_publisher',
            executable='gps_transform',
            name='gps_transform',
            output='screen'
        ),
        
        # base_footprint → base_link 변환 담당
        launch_ros.actions.Node(
            package='gps_imu_tf_publisher',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen'
        ),

        # IMU 데이터를 활용하여 odom → base_footprint 변환을 수행
        launch_ros.actions.Node(
            package='gps_imu_tf_publisher',
            executable='imu_to_odom',
            name='imu_to_odom',
            output='screen'
        ),
    ])
