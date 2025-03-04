import socket
import time
import rclpy
from rclpy.node import Node
import threading
import json
import errno
from hardcarry_interface.srv import EmergencyStop
from std_msgs.msg import Int16
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class RPCommunicationNode(Node):
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
    BUFFER_SIZE = 1024

    def __init__(self):
        super().__init__('rp_communication_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        self.host = '192.168.1.101'
        self.port = 8081
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.running = True
        self.connected = False
        
        self.subscription = self.create_subscription(
            Int16,
            '/robot_mode',
            self.robot_mode_callback,
            10
        )
        self.get_logger().info('robot_mode 토픽 구독 완료')
        
        self.emergency_stop_client = self.create_client(EmergencyStop, 'emergency_stop')
        if self.emergency_stop_client is None:
            self.get_logger().error('Emergency Stop 서비스 클라이언트 생성 실패')
        self.get_logger().info('Emergency Stop 서비스 클라이언트 생성 성공')
        
        # 테스트 모드 설정
        self.test_modes = list(self.MODE_MESSAGES.keys())
        self.current_mode_index = 0
        self.test_mode = self.declare_parameter('test_mode', False).value
        self.get_logger().warn(f'테스트 모드 파라미터: {self.test_mode}')
        
        if not self.init_server():
            self.get_logger().error('서버 초기화 실패')
            return
        
        if self.test_mode:
            self.get_logger().warn('테스트 모드가 활성화되었습니다')
            self.test_thread = threading.Thread(target=self.test_loop)
            self.test_thread.daemon = True
            self.test_thread.start()
        else:
            self.get_logger().info('테스트 모드가 비활성화되어 있습니다')

    def init_server(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.get_logger().info(f'서버가 {self.host}:{self.port}에서 시작되었습니다')
            
            accept_thread = threading.Thread(target=self.accept_connections)
            accept_thread.daemon = True
            accept_thread.start()
            
            return True
        except Exception as e:
            self.get_logger().error(f'서버 초기화 실패: {str(e)}')
            return False

    def accept_connections(self):
        while self.running:
            try:
                client_socket, client_address = self.server_socket.accept()
                self.get_logger().warn(f'새로운 클라이언트가 연결됨: {client_address}')
                
                if self.client_socket:
                    self.cleanup_client()
                
                self.client_socket = client_socket
                self.client_address = client_address
                self.connected = True
                
                receive_thread = threading.Thread(target=self.receive_message)
                receive_thread.daemon = True
                receive_thread.start()
                
            except Exception as e:
                self.get_logger().error(f'클라이언트 연결 수락 중 오류: {str(e)}')
                time.sleep(1)

    def receive_message(self):
        while self.running and self.connected:
            try:
                data = self.client_socket.recv(self.BUFFER_SIZE)
                if not data:
                    self.get_logger().warn('클라이언트가 연결을 종료했습니다')
                    break
                
                try:
                    message = json.loads(data.decode('utf-8'))
                    self.get_logger().info(f'수신된 메시지: {message}')
                    self.process_message(message)
                except json.JSONDecodeError:
                    self.get_logger().warning('잘못된 JSON 형식')
                    
            except ConnectionResetError:
                self.get_logger().warn('클라이언트가 강제로 연결을 종료했습니다')
                break
            except ConnectionAbortedError:
                self.get_logger().warn('연결이 중단되었습니다')
                break
            except Exception as e:
                self.get_logger().error(f'메시지 수신 중 오류: {str(e)}')
                break
        
        self.get_logger().info('클라이언트 연결 종료, 정리 작업 시작')
        self.cleanup_client()
        self.get_logger().info('클라이언트 연결 정리 완료')

    def send_message(self, payload):
        if not self.connected or not self.client_socket:
            return False
        try:
            message = json.dumps(payload).encode('utf-8')
            self.client_socket.send(message)
            return True
        except Exception as e:
            self.get_logger().error(f'메시지 전송 중 오류: {str(e)}')
            self.cleanup_client()
            return False
    
    def make_payload(self, action, mode, message):
        payload = {
            "action": action,
            "mode": mode,
            "message": message
        }
        return payload

    def process_message(self, formatted_message):
        try:
            action = formatted_message.get('action')
            mode = formatted_message.get('mode')
            message = formatted_message.get('message')
            
            if action == 'emergency_stop':
                self.emergency_stop()
            elif action == 'camera':
                self.camera_data(message)

        except Exception as e:
            self.get_logger().error(f'메시지 처리 중 오류: {str(e)}')

    def camera_data(self, image_data):
        self.get_logger().info(f'카메라 데이터 수신: {image_data}')

    def send_status_message(self, mode=0):
        try:
            mode = int(mode)
            message = self.MODE_MESSAGES.get(mode, 'unknown')
            if message == 'unknown':
                self.get_logger().error(f'잘못된 모드: {mode}')
                return False
            
            payload = self.make_payload("change_mode", mode, self.MODE_MESSAGES[mode])
            return self.send_message(payload)
            
        except ValueError:
            self.get_logger().error(f'잘못된 모드: {mode}')
            return False

    def emergency_stop(self):
        request = EmergencyStop.Request()
        request.emergency_stop = True
        future = self.emergency_stop_client.call_async(request)
        future.add_done_callback(self.handle_emergency_stop_response)

    def handle_emergency_stop_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().warn('비상 정지 절차 시행')
                self.cleanup()
                self.destroy_node()
                rclpy.shutdown()
            else:
                self.get_logger().error('비상 정지 실패')
        except Exception as e:
            self.get_logger().error(f'Emergency stop service call failed: {str(e)}')

    def robot_mode_callback(self, msg):
        self.get_logger().info('--------------------')
        self.get_logger().info(f'수신된 모드: {msg.data}')
        
        try:
            if self.send_status_message(msg.data):
                self.get_logger().info(f'모드 {msg.data} 전송 성공')
            else:
                self.get_logger().error(f'모드 {msg.data} 전송 실패')
        except Exception as e:
            self.get_logger().error(f'RobotMode 콜백 처리 중 오류: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        self.get_logger().info('--------------------')

    def test_loop(self):
        while self.running:
            if self.connected:
                self.test_timer_callback()
            time.sleep(3.0)

    def test_timer_callback(self):
        self.get_logger().info('테스트 타이머 콜백 시작')
        
        if not self.connected:
            self.get_logger().warn('클라이언트 연결 없음')
            return
        
        try:
            mode = self.test_modes[self.current_mode_index]
            self.get_logger().info(f'현재 테스트 모드: {mode}, 인덱스: {self.current_mode_index}')
            
            if self.send_status_message(mode):
                self.get_logger().info(f'테스트 모드 {mode} 전송 성공')
                self.current_mode_index = (self.current_mode_index + 1) % len(self.test_modes)
            else:
                self.get_logger().error(f'테스트 모드 {mode} 전송 실패')
            
        except Exception as e:
            self.get_logger().error(f'테스트 타이머 콜백 처리 중 오류: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def cleanup_client(self):
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            self.client_address = None
            self.connected = False

    def cleanup(self):
        self.running = False
        self.cleanup_client()
        if self.server_socket:
            self.server_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = RPCommunicationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('키보드 인터럽트 감지. 종료를 시작합니다...')
    finally:
        node.get_logger().info('정리 작업 시작')
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('정상 종료 완료')

if __name__ == '__main__':
    main()