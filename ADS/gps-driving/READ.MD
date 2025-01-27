# GPS,IMU데이터기반 Waypoints 주행

### Update Log
- Feat(25-01-27): Way-points 방향으로 Throttle, Steering 값을 오린카 기준으로 주행. 오린카 작동이 불가하여 스마트폰 움직임에 따라 모터 값을 Publish함.

### Topic 정보
**GPS 데이터**
ros2 topic echo /gps/fix

**IMU 데이터**
ros2 topic echo /imu/data

**제어 명령**
ros2 topic echo /cmd_ackermann
이 토픽에서 확인할 수 있는 값들:
throttle: -1.0 ~ 1.0 (전진/후진)
steering: 0.0 ~ 180.0 (조향각, 90도가 중립)