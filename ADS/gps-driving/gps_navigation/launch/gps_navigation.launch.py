# launch/gps_navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   
   config_dir = os.path.join(
        get_package_share_directory('gps_navigation'),
        'config',
        'waypoints.csv'
    )
   
   return LaunchDescription([
    # GPS IMU Publisher Node
      Node(
           package='gps_imu_publisher',
           executable='gps_imu_publisher_node',
           name='gps_imu_publisher',
           output='screen'
       ),

       # Waypoint Follower Node
       Node(
           package='gps_navigation',
           executable='waypoint_follower',
           name='waypoint_follower',
           parameters=[{
               'waypoints_file': config_dir,
               'goal_tolerance': 2.0
           }],
           output='screen'
       )
   ])