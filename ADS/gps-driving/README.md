# GPS 내비게이션 시스템

## 디렉토리 구조
nav_ws/src/
├── custom_interfaces/    # 모터 제어 메시지 정의 (throttle, steering)
├── gps_imu_publisher/   # 스마트폰에서 소켓통신으로 GPS/IMU 데이터 수신
├── gps_navigation/      # GPS 데이터 기반 모터 제어
└── motor_control/       # 기본 모터 제어 예제

## 실행 방법
ros2 launch gps_navigation gps_navigation.launch.py

## 토픽

### 수신 토픽
- `/gps/fix` : 스마트폰 GPS 위치 데이터
- `/imu/data` : 스마트폰 IMU 센서 데이터

### 발행 토픽
- `/cmd_ackermann` : 차량 제어 명령
  - throttle: 모터 출력 제어 (-1.0 ~ 1.0)
  - steering: 조향각 제어 (0.0 ~ 180.0도)

## 토픽 모니터링
# GPS 데이터 확인
ros2 topic echo /gps/fix

# IMU 데이터 확인
ros2 topic echo /imu/data

# 모터 제어 명령 확인
ros2 topic echo /cmd_ackermann