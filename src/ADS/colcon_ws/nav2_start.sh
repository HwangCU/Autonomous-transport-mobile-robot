#! /bin/bash

source /opt/ros/humble/setup.bash

COLCON_PATH=$(pwd)
MAP_PATH="/map.yaml"
FILE_PATH=$COLCON_PATH$MAP_PATH
echo $FILE_PATH

source $COLCON_PATH/install/setup.bash

ros2 launch stella_navigation2 navigation2.launch.py map:=$FILE_PATH
