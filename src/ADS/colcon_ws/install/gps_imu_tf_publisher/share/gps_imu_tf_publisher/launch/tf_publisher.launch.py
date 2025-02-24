import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='gps_imu_tf_publisher',
            executable='gps_transform',
            name='gps_transform'
        ),
        launch_ros.actions.Node(
            package='gps_imu_tf_publisher',
            executable='tf_publisher',
            name='tf_publisher'
        ),
        launch_ros.actions.Node(
            package='gps_imu_tf_publisher',
            executable='imu_to_odom',
            name='imu_to_odom'
        ),
    ])
