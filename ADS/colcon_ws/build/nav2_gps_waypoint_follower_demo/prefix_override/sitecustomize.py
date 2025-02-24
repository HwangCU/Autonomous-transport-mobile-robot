import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yun/colcon_ws/install/nav2_gps_waypoint_follower_demo'
