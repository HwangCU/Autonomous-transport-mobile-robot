import asyncio
import socket
import time
from PyQt5.QtCore import QObject, pyqtSignal
import json

class JetsonCommunication(QObject):
    message_received = pyqtSignal(str)  # 메시지 수신 시그널
    connection_status_changed = pyqtSignal(bool)  # 통신 연결 상태 변경 시그널

    def __init__(self, server_ip, server_port=8081):
        super().__init__()
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.is_connected = False
        self.max_retries = 3
        self.connect_timeout = 5  # 연결 타임아웃(sec)
        self.operation_timeout = 2  # 작업 타임아웃(sec)
        self.reconnect_interval = 3  # 재연결 시도 간격(sec)
        self.logger_prefix = "[JetsonComm]"

    def _update_connection_status(self, status):
        self.is_connected = status
        self.connection_status_changed.emit(status)

    async def connect(self):
        try:
            async with asyncio.timeout(self.connect_timeout):
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.server_port))
                self.socket.setblocking(False)
                self._update_connection_status(True)
                print(f'{self.logger_prefix} Connected to {self.server_ip}:{self.server_port}')
                asyncio.create_task(self._receive_data())
                return True
        except (asyncio.TimeoutError, Exception) as e:
            self._update_connection_status(False)
            print(f'{self.logger_prefix} Connection error: {e}')
            return False

    async def _reconnect(self):
        print(f"{self.logger_prefix} Attempting to reconnect...")
        while not self.is_connected:
            if await self.connect():
                print(f"{self.logger_prefix} Reconnection successful")
                break
            print(f"{self.logger_prefix} Reconnection failed, retrying in {self.reconnect_interval} seconds...")
            await asyncio.sleep(self.reconnect_interval)

    async def _receive_data(self):
        while True:
            try:
                if not self.is_connected:
                    await self._reconnect()
                    continue

                data = await asyncio.get_event_loop().sock_recv(self.socket, 1024)
                if not data:
                    print(f'{self.logger_prefix} Server와 연결이 끊어졌습니다.')
                    self._update_connection_status(False)
                    continue
                
                data = json.loads(data.decode('utf-8'))
                if not isinstance(data, dict):
                    raise ValueError("메시지가 JSON 객체 형식이 아닙니다")
                
                message = data.get('status')
                if message is None:
                    raise ValueError("status 필드가 없습니다")
                
                print(f'{self.logger_prefix} Received from server: {message}')
                self.message_received.emit(message)

            except Exception as e:
                print(f'{self.logger_prefix} Receive error: {e}')
                self._update_connection_status(False)
                await asyncio.sleep(1)  # 에러 발생 시 잠시 대기

    async def send_data(self, message, retries=0):
        if not self.is_connected:
            print(f"{self.logger_prefix} Not connected to server")
            return False
        
        try:
            async with asyncio.timeout(self.operation_timeout):
                payload = json.dumps({
                    "action": "send_message",
                    "mode": 700,
                    "status": message
                }).encode('utf-8')
                await asyncio.get_event_loop().sock_sendall(self.socket, payload)
                print(f"{self.logger_prefix} Message sent: {message}")
                return True
        except asyncio.TimeoutError:
            print(f"{self.logger_prefix} Send timeout: {message}")
        except Exception as e:
            print(f'{self.logger_prefix} Send error: {e}')
            if retries < self.max_retries:
                print(f'{self.logger_prefix} Retrying... ({retries + 1}/{self.max_retries})')
                await asyncio.sleep(1)
                return await self.send_data(message, retries + 1)
        return False

    async def emergency_stop(self):
        print("Emergency_stop")
        return await self.send_data('emergency_stop')

    async def close(self):
        self._update_connection_status(False)
        if self.socket:
            await asyncio.get_event_loop().sock_close(self.socket)
