import socket
import time
import rclpy
from rclpy.node import Node
import threading
import errno
import json
import asyncio
from concurrent.futures import ThreadPoolExecutor
from hardcarry_interface.msg import RobotMode
from hardcarry_interface.srv import EmergencyStop

class RPCommunicationNode(Node):
    # 메시지 모드 종류
    # mode : 0   대기 모드
    # mode : 1   조이콘 모드
    # mode : 2   팔로잉 모드
    # mode : 4   집으로 돌아가기 중일 때
    # mode : 41  집에 도착했을 때
    # mode : 3   배달 모드 - 배달 중
    # mode : 31  배달 모드 - 주문한 가게 도착
    # mode : 32  배달 모드 - 배달 완료
    # mode : 100 에러 상태
    # mode : 127 테스트 모드    
    MODE_MESSAGES = {
        0: 'standard',
        1: 'joycon',
        2: 'follow',
        3: 'delivery',
        31: 'delivery-stopover',
        32: 'delivery-arrived',
        4: 'gohome',
        41: 'home',
        100: 'error',
        127: 'test',
    }
    
    BUFFER_SIZE = 4096
    MAX_QUEUE_SIZE = 100
    
    def __init__(self):
        super().__init__('rp_communication_node')
        self.host = '192.168.1.101'
        self.port = 8081
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.connected = False
        self.running = True
        self.lock = threading.Lock()
        
        self.test_modes = list(self.MODE_MESSAGES.keys())
        self.current_mode_index = 0
        # ros2 run hardcarry rp_com_node --ros-args -p test_mode:=true
        self.test_mode = self.declare_parameter('test_mode', False).value
        
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        self.thread_pool = ThreadPoolExecutor(max_workers=3)
        self.message_queue = asyncio.Queue(maxsize=self.MAX_QUEUE_SIZE)
        
        self.init_server()
        self.process_task = self.loop.create_task(self.process_message_queue())
        
        if self.test_mode:
            # 테스트용
            self.create_timer(1.0, self.test_timer_callback)
        
        self.get_logger().info('RP Communication Server Node has been started.')
        
        # RobotMode 토픽 $구독$
        self.subscription = self.create_subscription(
            RobotMode,
            'RobotMode',
            self.robot_mode_callback,
            10
        )
        self.get_logger().info('RobotMode.msg 구독 시작')

        # Emergency Stop Service Client
        self.emergency_stop_client = self.create_client(EmergencyStop, 'emergency_stop')
        while not self.emergency_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Emergency Stop service not available, waiting...')
        self.get_logger().info('Emergency Stop service client created')

    def cleanup(self):
        self.running = False
        
        # 태스크 취소 먼저
        if hasattr(self, 'process_task'):
            self.process_task.cancel()
        
        # 스레드 정리
        if hasattr(self, 'accept_thread'):
            self.accept_thread.join(timeout=1.0)
        if hasattr(self, 'loop_thread'):
            self.loop_thread.join(timeout=1.0)
        
        # 나머지 리소스 정리
        self.cleanup_client()
        
        # 메시지 큐 정리
        if not self.message_queue.empty():
            self.get_logger().warning('메시지 큐에 처리되지 않은 메시지가 있습니다')
        
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception as e:
                self.get_logger().error(f'서버 소켓 정리 중 오류: {str(e)}')
                
        self.server_socket = None
        
        # 스레드 풀 정리
        try:
            self.thread_pool.shutdown(wait=True)
        except Exception as e:
            self.get_logger().error(f'스레드 풀 정리 중 오류: {str(e)}')

    def cleanup_client(self):
        with self.lock:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except Exception as e:
                    self.get_logger().error(f'클라이언트 소켓 닫기 중 오류: {str(e)}')
            self.client_socket = None
            self.client_address = None
            self.connected = False

    def init_server(self):
        try:
            # 기존 소켓이 있다면 정리
            if self.server_socket:
                self.server_socket.close()
                self.server_socket = None
            
            self.get_logger().info('소켓 생성 시작...')
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 블로킹 모드 확인
            self.get_logger().info(f'소켓 블로킹 모드: {self.server_socket.getblocking()}')
            
            # 추가 디버그 정보
            self.get_logger().info('현재 서버 설정:')
            self.get_logger().info(f'- Host: {self.host}')
            self.get_logger().info(f'- Port: {self.port}')
            
            # 바인드 전 소켓 상태 확인
            try:
                self.server_socket.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                self.get_logger().info('소켓 상태: 정상')
            except socket.error as e:
                self.get_logger().error(f'소켓 상태 확인 실패: {e}')
            
            self.get_logger().info('바인딩 시도...')
            try:
                self.server_socket.bind((self.host, self.port))
                self.get_logger().info('바인딩 성공')
            except socket.error as e:
                if e.errno == errno.EADDRINUSE:
                    self.get_logger().error('포트가 이미 사용 중입니다!')
                raise e
            
            self.get_logger().info('리스닝 시작...')
            self.server_socket.listen(1)
            self.get_logger().info('서버가 성공적으로 시작되었습니다.')
            
            # 소켓 정보 출력
            sock_name = self.server_socket.getsockname()
            self.get_logger().info(f'서버 소켓 정보: {sock_name}')
            
            # 클라이언트 연결을 기다리는 스레드 시작
            self.accept_thread = threading.Thread(target=self.accept_connections)
            self.accept_thread.daemon = True
            self.accept_thread.start()
            
            # 비동기 이벤트 루프 처리 스레드 실행
            self.loop_thread = threading.Thread(target=self._run_event_loop)
            self.loop_thread.daemon = True
            self.loop_thread.start()
            
            self.server_socket.settimeout(60)  # 60초 타임아웃
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'서버 초기화 실패: {str(e)}')
            self.get_logger().error(f'에러 타입: {type(e).__name__}')
            self.get_logger().error(f'상세 에러: {str(e)}')
            
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
                self.server_socket = None
            
            return False

    def _run_event_loop(self):
        self.loop.run_forever()

    def accept_connections(self):
        self.get_logger().info('클라이언트 연결 대기를 시작합니다...')
        while self.running:
            try:
                client_socket, client_address = self.server_socket.accept()
                self.get_logger().info(f'새로운 클라이언트가 연결됨: {client_address}')
                
                # 기존 클라이언트 정리
                if self.client_socket:
                    self.cleanup_client()
                
                self.client_socket = client_socket
                self.client_address = client_address
                self.connected = True
                
                # 메시지 수신 스레드 시작
                receive_thread = threading.Thread(target=self.handle_client)
                receive_thread.daemon = True
                receive_thread.start()
                
            except Exception as e:
                if self.running:  # 정상 종료가 아닌 경우만 에러 출력
                    self.get_logger().error(f'클라이언트 연결 수락 중 오류: {str(e)}')
                time.sleep(1)

    def handle_client(self):
        buffer = b''
        try:
            while self.connected and self.client_socket:
                try:
                    chunk = self.client_socket.recv(self.BUFFER_SIZE)
                    if not chunk:
                        self.get_logger().info('클라이언트가 연결을 종료했습니다.')
                        break
                    
                    buffer += chunk
                    try:
                        data = json.loads(buffer.decode('utf-8'))
                        buffer = b''
                        
                        if not isinstance(data, dict):
                            raise ValueError("메시지가 JSON 객체 형식이 아닙니다")
                        
                        message = data.get('status')
                        if message is None:
                            raise ValueError("status 필드가 없습니다")
                        
                        self.get_logger().info(f'수신된 데이터: {data}')
                        
                        if message == 'emergency_stop':
                            self.emergency_stop()
                        
                        asyncio.run_coroutine_threadsafe(
                            self.message_queue.put({
                                "action": "echo",
                                "mode": 800,
                                "status": message
                            }),
                            self.loop
                        )
                        
                    except json.JSONDecodeError:
                        # 완전한 JSON 메시지가 아직 수신되지 않았으면 계속 수신
                        continue
                    except Exception as e:
                        self.get_logger().error(f'메시지 파싱 오류: {str(e)}')
                        buffer = b''
                        
                except Exception as e:
                    self.get_logger().error(f'메시지 수신 중 오류: {str(e)}')
                    break
            
            self.cleanup_client()
        except Exception as e:
            self.get_logger().error(f'클라이언트 메시지 수신 중 오류: {str(e)}')
        finally:
            self.cleanup_client()

    def send_message(self, payload):
        with self.lock:
            if not self.connected or not self.client_socket:
                self.get_logger().error('클라이언트가 연결되어 있지 않습니다')
                return False
                
            try:
                message = json.dumps(payload).encode('utf-8')
                self.client_socket.send(message)
                self.get_logger().info(f'전송된 메시지: {payload}')
                return True
            except Exception as e:
                self.get_logger().error(f'메시지 전송 중 오류: {str(e)}')
                self.cleanup_client()
                return False

    def send_status_message(self, mode=0):
        message = self.MODE_MESSAGES.get(mode, 'unknown')
        if message == 'unknown':
            self.get_logger().error(f'알 수 없는 모드: [{mode}]')
            return
        
        payload = {
            "action": "change_status",
            "mode": mode,
            "status": message,
            "robot_id": "user"
        }
        
        asyncio.run_coroutine_threadsafe(
            self.message_queue.put(payload),
            self.loop
        )
        self.get_logger().info(f'상태 메시지 [{mode}] 큐에 추가됨')

    def emergency_stop(self):
        self.get_logger().info('!!!!!! Warning !!!!!!: Emergency Stop!')
        
        # Create and send emergency stop request
        request = EmergencyStop.Request()
        request.emergency_stop = True
        
        try:
            future = self.emergency_stop_client.call_async(request)
            
            # Wait for the response using a separate thread to avoid blocking
            def handle_emergency_stop_response(future):
                try:
                    response = future.result()
                    if response.success:
                        self.get_logger().info('Emergency stop request successful')
                        self.send_status_message(mode=127)
                    else:
                        self.get_logger().error(f'Emergency stop failed: {response.message}')
                except Exception as e:
                    self.get_logger().error(f'Emergency stop service call failed: {str(e)}')
            
            future.add_done_callback(handle_emergency_stop_response)
            
        except Exception as e:
            self.get_logger().error(f'Failed to send emergency stop request: {str(e)}')

    def test_timer_callback(self):
        if self.connected and self.client_socket:
            mode = self.test_modes[self.current_mode_index]
            self.send_status_message(mode)
            self.current_mode_index = (self.current_mode_index + 1) % len(self.test_modes)

    async def process_message_queue(self):
        while self.running:
            try:
                message = await self.message_queue.get()
                try:
                    # 스레드 풀에서 메시지 전송
                    await self.loop.run_in_executor(
                        self.thread_pool,
                        self.send_message,
                        message
                    )
                finally:
                    # 메시지 처리 완료 표시
                    self.message_queue.task_done()
            except Exception as e:
                self.get_logger().error(f'메시지 큐 처리 중 오류: {str(e)}')

    # RobotMode 토픽 메시지 수신 시 콜백 함수
    def robot_mode_callback(self, msg):
        self.get_logger().info(f'RobotMode 메시지 수신: mode={msg.mode}')
        self.send_status_message(msg.mode)

def main(args=None):
    rclpy.init(args=args)
    node = RPCommunicationNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
