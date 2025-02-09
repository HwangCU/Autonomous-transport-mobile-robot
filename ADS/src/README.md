# Navigation GPS Localization 커스텀 작업 중
### 실행 플로우
1. 실제 하드웨어 드라이버 실행 (귀하의 실제 센서 드라이버)\
```ros2 launch your_hardware_package your_hardware.launch.py```

2. 메시지 변환/리매핑을 위한 launch 파일 실행\
```ros2 launch nav2_gps_waypoint_follower_demo real_hardware.launch.py```

3. robot_localization 실행\
```ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat.launch.py```
4. Nav2 실행\
```ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py```


### 작업 로그
2025.02.09
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
2025.02.10