# 로봇 정보
## 실행방법
1. 로봇 브링업(Embedded)
```
ros2 launch stella_bringup robot.launch.py 
```
2. GPS신호 활성화(PC)
```
ros2 run gps localization_launch.py 
```
3. GPS Nav2 시스템 및 Mapviz 시작(PC)
```
ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py use_mapviz:=true
```
4-1. Mapviz에 click point로 자율주행
```
ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower
```
4-2. 미리 저장된 gps-waypoint로 주행
```
ros2 run nav2_gps_waypoint_follower_demo logged_waypoint_follower
```

