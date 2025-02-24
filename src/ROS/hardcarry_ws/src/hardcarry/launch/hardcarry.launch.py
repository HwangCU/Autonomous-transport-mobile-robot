from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    # driving_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.joing('path~~~')
    #     )
    # )
    
    
    return LaunchDescription([
        # driving_launch,
        Node(
            package='hardcarry',
            executable='following_node',
            name='following_node',
        ),
        Node(
            package='hardcarry',
            executable='main_node',
            name='main_node',
        ),

    ])