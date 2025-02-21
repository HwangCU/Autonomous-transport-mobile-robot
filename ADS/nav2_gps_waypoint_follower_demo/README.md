# Navigation GPS Localization 커스텀

## 구현 내용
1. dual_ekf_navsat_params.yaml 수정
- 엔코더 없는 상황에 맞게 EKF 파라미터 조정
- IMU 설정 최적화
- GPS 관련 파라미터 설정

2. real_hardware.launch.py 생성
- 실제 하드웨어를 위한 기본 설정
- URDF 발행
- 센서 TF tree 설정


3. gps_waypoint_follower.launch.py 수정
- 시뮬레이션 코드 제거
- 실제 하드웨어 설정 적용
- Nav2 파라미터 설정
---

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

