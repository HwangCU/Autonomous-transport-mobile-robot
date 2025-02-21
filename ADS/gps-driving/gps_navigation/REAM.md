### 파일 디렉토리 구조
~/chichi/gps_navigation/src/gps_navigation/      # 메인 패키지 디렉토리
├── gps_navigation/                       # 파이썬 패키지 디렉토리
│   ├── __init__.py
│   ├── utils/                           # 유틸리티 모듈
│   │   ├── __init__.py
│   │   └── coordinate_utils.py          # 좌표 변환 유틸리티
│   ├── controllers/                     # 컨트롤러 모듈
│   │   ├── __init__.py
│   │   └── pure_pursuit.py             # Pure Pursuit 구현
│   └── waypoint_follower.py            # 메인 노드
├── config/                              # 설정 파일들
│   ├── waypoints.csv                    # 웨이포인트 데이터
│   └── navigation_params.yaml           # 네비게이션 파라미터
├── launch/                              # 런치 파일
│   └── gps_navigation.launch.py         # 실행 설정
├── package.xml                          # 패키지 정보
├── setup.py                             # 패키지 설정
└── setup.cfg                            # 패키지 설정

### Python 패키지 설치
pip3 install pyproj numpy transforms3d

waypoints.csv 파일을 생성하고 테스트용 GPS 좌표를 입력해야 합니다.
테스트 순서:
현재 로봇 위치의 GPS 좌표 확인
```
Copyros2 topic echo /gps/fix
```
목적지 GPS 좌표 수집
로봇이 이동할 경로의 주요 지점 GPS 좌표 기록
waypoints.csv 작성:
csvCopylatitude,longitude,speed
[시작점_위도],[시작점_경도],0.3
[중간점_위도],[중간점_경도],0.3
[목적지_위도],[목적지_경도],0.3

### GPS 네비게이션 테스트 단계:

1. 노드 실행
```bash
ros2 launch gps_navigation gps_navigation.launch.py
```

2. 토픽 모니터링 (새 터미널에서)
```bash
# GPS 데이터
ros2 topic echo /gps/fix

# IMU 데이터
ros2 topic echo /imu/data

# 제어 명령
ros2 topic echo /cmd_ackermann
```

3. 로그 모니터링
```bash
ros2 run rqt_console rqt_console
```