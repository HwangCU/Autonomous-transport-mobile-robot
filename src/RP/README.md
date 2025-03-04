# 메시지 포맷
## 화면 전환 메시지
```json
{
    "action": "change_mode",
    "mode": 0,
    "message": "message"
}
```
### mode 종류
- 0 : 대기 모드
- 1 : 조이콘 모드
- 2 : 팔로잉 모드
- 4 : 집으로 돌아가기 중일 때
- 41 : 집에 도착했을 때
- 3 : 배달 모드 - 배달 중
- 31 : 배달 모드 - 주문한 가게 도착
- 32 : 배달 모드 - 배달 완료
- 100 : 에러 상태
- 127 : 테스트 모드
## 비상정지 메시지
```json
{
    "action": "emergency_stop",
    "mode": 999,
    "message": "emergency_stop"
}
```
## 카메라 데이터 메시지
```json
{
    "action": "camera",
    "mode": 300,
    "message": "image_data"
}
```
## 소켓 통신 응답 메시지
```json
{
    "action": "echo",
    "mode": 800,
    "message": "message"
}
```