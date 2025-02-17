import websocket
import json
import ssl
import os
import threading
import time
from vars import SERVER_ADDR, SERVER_PORT


class WSManager:
    def __init__(self, url):
        self.url = url
        self.ws = None  # 아직 연결되지 않은 상태
        self.cert_path = "./fullchain.pem"  # 서버 인증서 경로 (예: './cert.pem')

    def is_connected(self):
        return self.ws is not None

    def connect(self):
        """
        - 최초로 호출되면 서버에 연결을 맺고 self.ws에 보관
        - 이미 연결돼 있으면 재연결하지 않음
        """
        if not self.ws:
            # 1) TLS 클라이언트 모드 SSLContext 생성
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)

            # 2) 서버 인증서를 신뢰 목록으로 등록
            if os.path.exists(self.cert_path):
                ssl_context.load_verify_locations(cafile="./fullchain.pem")
            else:
                print(
                    f"WARNING: {self.cert_path} not found. Will fail if cert is required."
                )

            # 3) WebSocket 연결
            print(f"[WSManager] Connecting to {self.url}...")
            self.ws = websocket.WebSocket()
            self.ws.connect(self.url, ssl=ssl_context)
            print("[WSManager] Connected.")

            # (옵션) 로컬 소켓 정보 확인
            local_info = self.ws.sock.getsockname()
            print(f"[WSManager] Local socket info: {local_info}")

            # ✅ 메시지 수신을 위한 별도 스레드 실행
            self.running = True
            threading.Thread(target=self.receive_messages, daemon=True).start()
            threading.Thread(target=self.send_ping, daemon=True).start()

    def send_ping(self):
        """서버와의 연결을 유지하기 위해 주기적으로 ping을 보냄"""
        while self.running:
            try:
                if self.ws and self.is_connected():
                    self.ws.ping()
                    print("[WSManager] Ping sent")
            except Exception as e:
                print(f"[WSManager] Ping failed: {e}")
                self.running = False
                break
            time.sleep(self.ping_interval)

    def receive_messages(self):
        """서버에서 메시지를 지속적으로 수신하는 함수"""
        while self.running:
            try:
                message = self.ws.recv()  # 서버로부터 메시지 수신
                self.on_message(message)  # 메시지 처리 함수 호출
            except Exception as e:
                print(f"[WSManager] Error in receive_messages: {e}")
                self.running = False
                break  # 오류 발생 시 루프 종료

    def on_message(self, message):
        """서버에서 메시지를 받았을 때 실행되는 콜백 함수"""
        try:
            data = json.loads(message)  # JSON 형식이면 파싱
            print(f"[WSManager] Received JSON message: {data}")
        except json.JSONDecodeError:
            print(f"[WSManager] Received raw message: {message}")

    def close(self):
        """
        - 필요에 따라 호출하면 연결을 해제
        """
        if self.ws:
            print("[WSManager] Closing connection.")
            self.ws.close()
            self.ws = None

    def send_login(self, payload, timeout=5):
        if not self.is_connected():
            print("[WSManager] Not connected yet.")
            return None

        try:
            self.ws.send(json.dumps(payload))
            resp_str = self.ws.recv()  # 한 번만 수신
            return json.loads(resp_str)
        except Exception as e:
            print(f"[WSManager] Error in send_login: {e}")
            return None


# 전역 싱글톤 인스턴스
ws_manager = WSManager(f"wss://{SERVER_ADDR}:{SERVER_PORT}")
