import websocket
import json
import ssl
import os
import threading
import queue
import socket
import time

# 예: 서버 주소/포트를 vars에서 가져온다고 가정
from vars import SERVER_ADDR, SERVER_PORT


########################################################################
# 1) WebSocket 클라이언트 매니저
########################################################################
class WSManager:
    def __init__(self, url):
        self.url = url
        self.ws = None
        self.cert_path = "./fullchain.pem"
        self.running = False
        self.message_queue = queue.Queue()

    def is_connected(self):
        return self.ws is not None

    def connect(self):
        if not self.ws:
            # TLS 클라이언트 모드 SSLContext
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            if os.path.exists(self.cert_path):
                ssl_context.load_verify_locations(cafile=self.cert_path)
            else:
                print(f"WARNING: {self.cert_path} not found.")

            print(f"[WSManager] Connecting to {self.url}...")
            self.ws = websocket.WebSocket()
            self.ws.connect(self.url, ssl=ssl_context)
            print("[WSManager] Connected.")
            self.running = True
            # 메시지 수신 스레드 시작
            threading.Thread(target=self.receive_messages, daemon=True).start()

    def receive_messages(self):
        """WebSocket 서버에서 오는 메시지를 계속 수신"""
        while self.running:
            try:
                message = self.ws.recv()
                # 1) 이 메시지를 큐에 넣거나, 바로 TCP로 전달
                self.message_queue.put(message)
                print(f"[WSManager] Received from WS: {message}")

                # 2) TCP로 전달
                broadcast_to_tcp_clients(message)

            except Exception as e:
                print(f"[WSManager] Error in receive_messages: {e}")
                self.running = False
                break

    def close(self):
        self.running = False
        if self.ws:
            print("[WSManager] Closing WS connection.")
            self.ws.close()
            self.ws = None

    def send_login(self, payload, timeout=5):
        """
        - 로그인 메시지를 보내고, 로그인 응답을 기다림
        - 응답이 올 때까지 `queue`에서 메시지를 대기함
        """
        if not self.is_connected():
            print("[WSManager] Not connected yet.")
            return None

        try:
            self.ws.send(json.dumps(payload))  # 🔹 로그인 요청 전송
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


########################################################################
# 2) TCP 소켓 서버
########################################################################
HOST = "0.0.0.0"
TCP_PORT = 9999

# 🔹 여러 클라이언트가 동시에 연결될 수 있으므로, 연결 목록을 전역 관리
connected_tcp_clients = []
lock = threading.Lock()


def handle_client(conn, addr):
    print(f"[TCP] Connected by {addr}")
    with lock:
        connected_tcp_clients.append(conn)

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            # TODO: TCP 클라이언트 → WebSocket 으로 보낼 수도 있음 (원하면)
            print(f"[TCP] Received from {addr}: {data.decode()}")
            # 여기서는 Echo 예시:
            conn.sendall(b"Server ack: " + data)
    except Exception as e:
        print("[TCP] Client error:", e)
    finally:
        conn.close()
        print(f"[TCP] Disconnected {addr}")
        with lock:
            connected_tcp_clients.remove(conn)


def run_tcp_server():
    print(f"[TCP] Starting TCP server on {HOST}:{TCP_PORT}")
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, TCP_PORT))
    server_sock.listen()

    while True:
        conn, addr = server_sock.accept()
        t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        t.start()


def broadcast_to_tcp_clients(message):
    """
    WebSocket에서 받은 메시지를
    연결된 모든 TCP 클라이언트에게 전송
    """
    print("연결된 클라이언트들에게 메시지 전송: ", message)
    with lock:
        for conn in connected_tcp_clients:
            try:
                # 단순히 문자열을 전송한다고 가정
                print("send message to client: ", message)
                conn.sendall(message.encode())
            except Exception as e:
                print("[TCP] Error sending to client:", e)


########################################################################
# 3) 전역 객체: WebSocket 매니저
########################################################################
ws_manager = WSManager(f"wss://{SERVER_ADDR}:{SERVER_PORT}")

########################################################################
# 4) 파일 단독 실행 시
########################################################################
if __name__ == "__main__":
    # 1) WebSocket 연결
    t_ws = threading.Thread(target=ws_manager.connect, daemon=True)
    t_ws.start()

    # 2) TCP 서버 실행
    t_tcp = threading.Thread(target=run_tcp_server, daemon=True)
    t_tcp.start()

    print("[MAIN] Both threads started. Press Ctrl+C to exit.")
    # 3) 메인 스레드 유지를 위해 대기
    while True:
        time.sleep(60)
