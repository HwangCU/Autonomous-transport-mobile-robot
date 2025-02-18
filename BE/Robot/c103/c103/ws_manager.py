import websocket
from websockets import connect
import json
import ssl
import os
import threading
import time
import queue  # 🔹 메시지 큐를 사용하여 동기화
from vars import SERVER_ADDR, SERVER_PORT


class WSManager:
    def __init__(self, url):
        self.url = url
        self.ws = None  # 아직 연결되지 않은 상태
        self.cert_path = "./fullchain.pem"  # 서버 인증서 경로 (예: './cert.pem')
        self.running = False
        self.message_queue = queue.Queue()  # 🔹 메시지를 저장할 큐 생성

    def is_connected(self):
        return self.ws is not None

    async def connect(self):
        """
        - 최초로 호출되면 서버에 연결을 맺고 self.ws에 보관
        - 이미 연결돼 있으면 재연결하지 않음
        """
        if not self.ws:
            # 1) TLS 클라이언트 모드 SSLContext 생성
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)

            # 2) 서버 인증서를 신뢰 목록으로 등록
            if os.path.exists(self.cert_path):
                ssl_context.load_verify_locations(cafile=self.cert_path)
            else:
                print(
                    f"WARNING: {self.cert_path} not found. Will fail if cert is required."
                )

            # 3) WebSocket 연결
            print(f"[WSManager] Connecting to {self.url}...")
            async with connect(self.url, ssl=ssl_context) as websocket:
                self.ws = websocket
                print("[WSManager] Connected.")

                # ✅ 메시지 수신을 위한 별도 스레드 실행
                self.running = True
                threading.Thread(target=self.receive_messages, daemon=True).start()

    async def receive_messages(self):
        """서버에서 메시지를 지속적으로 수신하는 함수"""
        while self.running:
            try:
                message = await self.ws.recv()  # 서버로부터 메시지 수신
                self.message_queue.put(message)  # 🔹 메시지를 큐에 저장
                print(f"[WSManager] Received and stored message: {message}")
            except Exception as e:
                print(f"[WSManager] Error in receive_messages: {e}")
                self.running = False
                break  # 오류 발생 시 루프 종료

    def close(self):
        """웹소켓 연결 종료"""
        self.running = False
        if self.ws:
            print("[WSManager] Closing connection.")
            self.ws.close()
            self.ws = None

    # await websocket.send(json.dumps(payload))
    async def send_login(self, payload, timeout=5):
        """
        - 로그인 메시지를 보내고, 로그인 응답을 기다림
        - 응답이 올 때까지 `queue`에서 메시지를 대기함
        """
        if not self.is_connected():
            print("[WSManager] Not connected yet.")
            return None

        try:
            await self.ws.send(json.dumps(payload))  # 🔹 로그인 요청 전송
            print("[WSManager] Sent login request, waiting for response...")

            # 🔹 지정된 timeout 동안 메시지 대기
            response = self.message_queue.get(timeout=timeout)
            print(f"[WSManager] Received login response: {response}")

            return json.loads(response)  # JSON 응답 반환
        except queue.Empty:
            print("[WSManager] Login response timeout!")
            return None
        except Exception as e:
            print(f"[WSManager] Error in send_login: {e}")
            return None


# 전역 싱글톤 인스턴스
ws_manager = WSManager(f"wss://{SERVER_ADDR}:{SERVER_PORT}")
