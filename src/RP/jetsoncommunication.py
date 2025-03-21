import asyncio
import socket
import time
from PyQt5.QtCore import QObject, pyqtSignal
import json
from typing import Optional

class JetsonCommunication(QObject):
    message_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)

    def __init__(self, server_ip: str, server_port: int = 8081) -> None:
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

    def _update_connection_status(self, status: bool) -> None:
        self.is_connected = status
        self.connection_status_changed.emit(status)

    async def connect(self) -> bool:
        try:
            async with asyncio.timeout(self.connect_timeout):
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.server_port))
                self.socket.setblocking(False)
                self._update_connection_status(True)
                print(f'{self.logger_prefix} Connected to {self.server_ip}:{self.server_port}')
                return True
        except (asyncio.TimeoutError, Exception) as e:
            self._update_connection_status(False)
            print(f'{self.logger_prefix} Connection error: {e}')
            if self.socket:
                self.socket.close()
                self.socket = None
            return False

    async def _reconnect(self) -> None:
        print(f"{self.logger_prefix} Attempting to reconnect...")
        retry_count = 0
        while not self.is_connected:
            retry_count += 1
            print(f"{self.logger_prefix} Reconnection attempt {retry_count}")
            if await self.connect():
                print(f"{self.logger_prefix} Reconnection successful")
                break
            print(f"{self.logger_prefix} Reconnection failed, retrying in {self.reconnect_interval} seconds...")
            await asyncio.sleep(self.reconnect_interval)

    async def _receive_data(self) -> None:
        while True:
            try:
                if not self.is_connected or not self.socket:
                    await self._reconnect()
                
                try:
                    data = await asyncio.get_event_loop().sock_recv(self.socket, 1024)
                    if not data:
                        raise ConnectionError("Connection lost")
                    
                    data = json.loads(data.decode('utf-8'))
                    if not isinstance(data, dict):
                        raise ValueError("메시지가 JSON 객체 형식이 아닙니다")

                    action = data.get('action')
                    mode = data.get('mode')
                    message = data.get('message')

                    if not all([action, mode is not None, message]):
                        raise ValueError("필수 필드가 누락되었습니다")

                    if action == 'camera':
                        # 카메라 데이터 처리는 별도로 구현
                        continue
                    elif action == 'echo':
                        print(f'{self.logger_prefix} Echo received: {message}')
                        if message == 'emergency_stop':
                            self.message_received.emit(message)
                        continue
                    elif action == 'change_mode':
                        self.message_received.emit(message)
                    
                    print(f'{self.logger_prefix} Received: action={action}, mode={mode}, message={message}')

                except (ConnectionError, json.JSONDecodeError) as e:
                    print(f'{self.logger_prefix} Connection/Data error: {e}')
                    self._update_connection_status(False)
                    if self.socket:
                        self.socket.close()
                        self.socket = None
                    await asyncio.sleep(1)

            except Exception as e:
                print(f'{self.logger_prefix} Receive error: {e}')
                self._update_connection_status(False)
                if self.socket:
                    self.socket.close()
                    self.socket = None
                await asyncio.sleep(1)

    async def send_data(self, action: str, mode: int, message: str, retries: int = 0) -> bool:
        if not self.is_connected:
            print(f"{self.logger_prefix} 서버와 연결되어 있지 않습니다. 재연결을 시도합니다.")
            await self._reconnect()
            if not self.is_connected:
                return False
        
        try:
            async with asyncio.timeout(self.operation_timeout):
                payload = json.dumps({
                    "action": action,
                    "mode": mode,
                    "message": message
                }).encode('utf-8')
                await asyncio.get_event_loop().sock_sendall(self.socket, payload)
                print(f"{self.logger_prefix} Message sent: {payload}")
                return True
        except asyncio.TimeoutError:
            print(f"{self.logger_prefix} Send timeout: {message}")
        except Exception as e:
            print(f'{self.logger_prefix} 전송 오류: {e}')
            self._update_connection_status(False)
            if retries < self.max_retries:
                print(f'{self.logger_prefix} Retrying... ({retries + 1}/{self.max_retries})')
                await asyncio.sleep(1)
                return await self.send_data(action, mode, message, retries + 1)
        return False

    async def emergency_stop(self) -> bool:
        print(f"{self.logger_prefix} Emergency stop")
        result = await self.send_data('emergency_stop', 999, 'emergency_stop')
        return result

    async def close(self) -> None:
        self._update_connection_status(False)
        if self.socket:
            await asyncio.get_event_loop().sock_close(self.socket)
