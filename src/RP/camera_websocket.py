import cv2
import time
import asyncio
import websockets
import json
import base64
from picamera2 import Picamera2

# WebSocket 서버 주소
SERVER_URL = "wss://c103.duckdns.org:8501"

# 연결 재시도 설정
RECONNECT_DELAY = 5  # 재연결 대기 시간(초)
FPS = 10  # 초당 프레임 수
FRAME_DELAY = 1 / FPS  # 프레임 간 지연 시간(초)
JPEG_QUALITY = 70  # JPEG 품질 (낮출수록 크기 감소, 품질 저하)

async def setup_camera():
    """카메라 초기화 및 설정"""
    picam2 = Picamera2()
    # 해상도 설정 - 필요에 따라 조정
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    # 카메라 안정화 대기
    await asyncio.sleep(2)
    return picam2

async def capture_and_send(websocket, picam2):
    """단일 프레임 캡처 및 전송"""
    try:
        # 프레임 캡처
        frame = picam2.capture_array()
        # RGB에서 BGR로 변환 (OpenCV 표준)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # JPEG으로 인코딩
        _, img_encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        
        # MJPEG 형식으로 변환
        mjpeg_part = (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(img_encoded)).encode() + b"\r\n\r\n"
            + img_encoded.tobytes() + b"\r\n"
        )
        
        # Base64 인코딩
        base64_mjpeg = base64.b64encode(mjpeg_part).decode('utf-8')
        
        # JSON 페이로드 생성 및 전송
        payload = json.dumps({
            "action": "rpi_image",
            "image": base64_mjpeg,
            "robot_id": "user"
        })
        
        await websocket.send(payload)
        return True
    except Exception as e:
        print(f"[ERROR] 프레임 처리 오류: {e}")
        return False

async def stream_video():
    """메인 비디오 스트리밍 함수"""
    # 카메라 초기화
    picam2 = await setup_camera()
    
    while True:
        try:
            print("[INFO] WebSocket 서버에 연결 중...")
            # 웹소켓 연결 - ping 비활성화
            async with websockets.connect(
                SERVER_URL, 
                ping_interval=None,
                ping_timeout=None,
                close_timeout=5
            ) as websocket:
                print("[INFO] 연결 성공. 스트리밍 시작...")
                
                # 프레임 스트리밍 루프
                while True:
                    start_time = time.time()
                    
                    # 프레임 캡처 및 전송
                    success = await capture_and_send(websocket, picam2)
                    if not success:
                        break
                    
                    # FPS 조절을 위한 지연
                    elapsed = time.time() - start_time
                    sleep_time = max(0, FRAME_DELAY - elapsed)
                    await asyncio.sleep(sleep_time)
        
        except Exception as e:
            print(f"[ERROR] 연결 실패: {e}")
            print(f"[INFO] {RECONNECT_DELAY}초 후 재연결 시도...")
            await asyncio.sleep(RECONNECT_DELAY)

# 메인 실행
if __name__ == "__main__":
    try:
        asyncio.run(stream_video())
    except KeyboardInterrupt:
        print("[INFO] 프로그램이 사용자에 의해 종료되었습니다.")
    except Exception as e:
        print(f"[ERROR] 예상치 못한 오류: {e}")