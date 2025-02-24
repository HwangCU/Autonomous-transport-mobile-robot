import sys
import asyncio
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QDesktopWidget, QLineEdit, QTextEdit
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtSvg import QSvgWidget
from loadhorize import LoadHorize, LoadHorizeTest
from jetsoncommunication import JetsonCommunication
from camera_class import CameraClass
import threading
import json
import os
import pathlib

# 비동기 작업을 처리하기 위한 스레드
class AsyncThread(QThread):
    signal = pyqtSignal(str)  # UI 업데이트를 위한 시그널

    def __init__(self, load_horize):
        super().__init__()
        self.load_horize = load_horize

    def run(self) -> None:
        asyncio.run(self.load_horize.main())

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.logger_prefix = "[RobotUI]"
        # 동적으로 변경될 UI 요소
        self.unlock_button = None
        self.status_label = None
        self.caution_widget = None
        self.test_button = None  # 테스트 버튼 참조 추가
        
        # Jetson 연결 설정
        self.jetson = JetsonCommunication(server_ip='192.168.1.101')
        self.jetson.connection_status_changed.connect(self.handle_connection_status)
        
        # LoadHorize 인스턴스 생성
        self.load_horize = LoadHorize()
        # self.load_horize = LoadHorizeTest()
        
        # 카메라 인스턴스 생성
        # self.camera = CameraClass()
        
        # 비동기 작업을 위한 스레드 설정
        self.async_thread = AsyncThread(self.load_horize)
        self.async_thread.start()
        
        self.connection_retry_count = 0
        self.retry_interval = 3  # 재시도 간격 (초)

        # 기본 설정 메시지
        self.default_message = "standard"
        
        self.loop = None  # loop를 None으로 초기화
        
        self.shutdown_script = str(pathlib.Path(__file__).parent / 'shutdown.sh')
        
        self.test_window = None  # 테스트 창 참조 추가
        
        self.initUI()
        self.update_ui_state(self.default_message)
        
    # Jetson 연결 초기화
    async def init_jetson(self) -> None:
        asyncio.create_task(self.jetson._receive_data())
        if await self.jetson.connect():
            self.jetson.message_received.connect(self.handle_jetson_message)
    
    # Jetson으로부터 수신한 메시지 처리하는 메서드
    def handle_jetson_message(self, message: str) -> None:
        if message == 'emergency_stop':
            self.handle_emergency_stop()
            self.update_ui_state(message)
        else:
            self.update_ui_state(message)
    
    # 비상정지 처리
    def handle_emergency_stop(self) -> None:
        try:
            print(f"{self.logger_prefix} 비상정지 신호 수신. 프로그램을 종료합니다.")
            self.update_ui_state('emergency_stop')
            
            # 비동기 작업 정리
            if self.loop and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.cleanup)
            
            # Qt 애플리케이션 종료
            QApplication.instance().quit()
            
            # 라즈베리파이 종료
            print(f"{self.logger_prefix} 라즈베리파이를 종료합니다...")
            os.system('sudo shutdown now')
            
        except Exception as e:
            print(f"{self.logger_prefix} 비상정지 처리 중 오류: {e}")
            try:
                # 라즈베리파이 강제 종료 시도
                os.system(f'sudo {self.shutdown_script}')
            except:
                pass
            sys.exit(1)

    # 비상 정지 버튼 클릭 시 실행
    def emergency_stop(self) -> None:
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.jetson.emergency_stop(),
                self.loop
            )
            if future.result():
                self.handle_emergency_stop()
        except Exception as e:
            print(f"{self.logger_prefix} 비상정지 명령 처리 중 오류: {e}")
            sys.exit(1)

    # 로봇 잠금 해제 메시지 전송
    def unlock_robot(self) -> None:
        try:
            # 비동기 함수를 동기적으로 실행
            asyncio.run_coroutine_threadsafe(
                self.jetson.send_data('test', 555, 'test'),
                self.loop
            )
            print(f"{self.logger_prefix} 잠금해제 메시지 전송 시도")
        except Exception as e:
            print(f"{self.logger_prefix} 잠금해제 명령 처리 중 오류: {e}")

    async def send_camera_image(self, image: bytes) -> None:
        await self.jetson.send_data('camera_image', image)

    def update_ui_state(self, signal: str) -> None:
        # 분기 처리
        if signal == 'standard':
            self.status_label.setText("안녕하세요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'joycon': # 조이콘 모드 (mode: 1)
            self.status_label.setText("조이콘 모드가 실행 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'follow': # 팔로잉 모드 (mode: 2)
            self.status_label.setText("팔로잉 모드가 실행 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'gohome': # 집으로 돌아가기 중일 때 (mode: 4)
            self.status_label.setText("집으로 돌아가고 있어요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'home': # 집에 도착했을 때 (mode: 41)
            self.status_label.setText("집에 도착했어요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'delivery': # 배달 모드 - 배달 중 (mode: 3)
            self.status_label.setText("로봇이 배달 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'delivery-stopover': # 배달 모드 - 주문한 가게 도착 (mode: 31)
            self.status_label.setText("로봇이 목적지에 도착했어요")
            self.unlock_button.show()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'delivery-arrived': # 배달 모드 - 배달 완료 (mode: 32)
            self.status_label.setText("배달을 완료했어요")
            self.unlock_button.hide()
            self.caution_widget.show()
            self.test_button.hide()
        elif signal == 'error': # 에러 상태 (mode: 100)
            self.status_label.setText("로봇에 문제가 발생했습니다")
            self.unlock_button.hide()
            self.caution_widget.hide()
            self.test_button.hide()
        elif signal == 'connection_error': # 젯슨과 통신 오류
            self.status_label.setText(f"로봇과 연결 중입니다... (시도 횟수: {self.connection_retry_count})")
            self.unlock_button.hide()
            self.caution_widget.hide()
            self.test_button.hide()
        elif signal == 'emergency_stop': # 비상정지 (mode: 999)
            self.status_label.setText("비상정지 중 입니다")
            self.unlock_button.hide()
            self.caution_widget.hide()
            self.test_button.hide()
        elif signal == 'test':  # 테스트 모드일 때만 테스트 버튼 표시
            self.status_label.setText("테스트 모드가 실행 중이에요")
            self.unlock_button.hide()
            self.caution_widget.hide()
            self.test_button.show()
        
    def handle_connection_status(self, is_connected: bool) -> None:
        if is_connected:
            self.connection_retry_count = 0
            self.update_ui_state(self.default_message)
        else:
            self.connection_retry_count += 1
            print(f"{self.logger_prefix} 연결 재시도... (시도 횟수: {self.connection_retry_count})")
            self.update_ui_state("connection_error")

    def initUI(self) -> None:
        # 화면 설정
        screen = QDesktopWidget().screenGeometry()
        window_width = int(screen.width())
        window_height = int(screen.height())
        
        self.setWindowTitle('Robot UI')
        self.setStyleSheet("background-color: #E5ECF0;")
        self.setFixedSize(window_width, window_height)
        self.center()
        
        # UI 요소 크기
        unlock_width = int(window_width * 0.4)
        button_height = int(window_height * 0.17)
        emergency_width = int(window_width * 0.21)
        
        # 중앙 위젯 및 메인 레이아웃 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setAlignment(Qt.AlignCenter)
        main_layout.setSpacing(20)
        
        # 상태 메시지
        self.status_label = QLabel()
        self.status_label.setStyleSheet("""
            color: #5E77E1;
            font-size: """ + str(int(window_height * 0.08)) + """px;
            font-weight: bold;
        """)
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)
        
        # 주의 메시지
        self.caution_widget = QWidget()
        caution_layout = QHBoxLayout(self.caution_widget)
        
        star_icon = QSvgWidget("./assets/star_filled.svg")
        star_icon.setFixedSize(int(window_height * 0.05), int(window_height * 0.05))
        
        caution_text = QLabel("자율주행을 위해 카메라가 주위를 보고 있어요")
        caution_text.setStyleSheet("""
            color: #5E77E1;
            font-size: """ + str(int(window_height * 0.04)) + """px;
            font-weight: bold;
        """)
        
        caution_layout.addWidget(star_icon)
        caution_layout.addWidget(caution_text)
        caution_layout.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.caution_widget)
        
        # 잠금해제 버튼을 위한 컨테이너
        unlock_container = QWidget()
        unlock_container.setFixedHeight(button_height)  # 높이 고정
        unlock_layout = QHBoxLayout(unlock_container)
        unlock_layout.setContentsMargins(0, 0, 0, 0)
        
        # 잠금해제 버튼
        self.unlock_button = QPushButton("로봇 잠금 해제")
        self.unlock_button.setFixedSize(unlock_width, button_height)
        self.unlock_button.setStyleSheet("""
            QPushButton {
                background-color: #F6DE79;
                border-radius: 15px;
                color: #636363;
                font-size: """ + str(int(window_height * 0.06)) + """px;
                font-weight: bold;
            }
        """)
        self.unlock_button.clicked.connect(self.unlock_robot)
        unlock_layout.addWidget(self.unlock_button, alignment=Qt.AlignCenter)
        main_layout.addWidget(unlock_container, alignment=Qt.AlignCenter)
        
        # 비상정지 버튼
        emergency_container = QWidget()
        emergency_layout = QHBoxLayout(emergency_container)
        emergency_layout.setContentsMargins(0, 0, 0, 0)
        
        emergency_button = QPushButton("비상정지")
        emergency_button.setFixedSize(emergency_width, button_height)
        emergency_button.setStyleSheet("""
            QPushButton {
                background-color: #E23C3C;
                border-radius: 15px;
                color: white;
                font-size: """ + str(int(window_height * 0.06)) + """px;
                font-weight: bold;
            }
        """)
        emergency_button.clicked.connect(self.emergency_stop)
        
        emergency_layout.addStretch()
        emergency_layout.addWidget(emergency_button)
        main_layout.addWidget(emergency_container)
        
        # 테스트 모드 버튼 추가
        self.test_button = QPushButton("테스트 모드")
        self.test_button.setStyleSheet("""
            QPushButton {
                background-color: #5E77E1;
                border-radius: 15px;
                color: white;
                font-size: 24px;
                font-weight: bold;
                padding: 10px;
            }
        """)
        self.test_button.clicked.connect(self.show_test_window)
        self.test_button.hide()
        main_layout.addWidget(self.test_button)
        
        # 여백 설정
        main_layout.setContentsMargins(50, 50, 50, 50)

    # 창을 화면 중앙에 위치시키는 메서드
    def center(self) -> None:
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def cleanup(self) -> None:
        try:
            self.load_horize.cleanup()
            if self.loop and self.loop.is_running():
                # 먼저 이벤트 루프의 모든 태스크를 정리
                pending = asyncio.all_tasks(self.loop)
                for task in pending:
                    task.cancel()
                
                # Jetson 연결 종료
                close_task = asyncio.run_coroutine_threadsafe(
                    self.jetson.close(),
                    self.loop
                )
                try:
                    close_task.result(timeout=2)  # 2초 타임아웃으로 변경
                except Exception as e:
                    print(f"Jetson close error: {e}")
                
                # 이벤트 루프 정리
                self.loop.call_soon_threadsafe(self.loop.stop)
                
        except Exception as e:
            print(f"Final cleanup error: {e}")

    def show_test_window(self):
        if not self.test_window:
            self.test_window = TestWindow(self.jetson, self.loop)
        self.test_window.show()

class TestWindow(QWidget):
    def __init__(self, jetson, loop):
        super().__init__()
        self.jetson = jetson
        self.loop = loop
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('테스트 모드')
        self.setStyleSheet("background-color: #E5ECF0;")
        
        layout = QVBoxLayout()
        
        # 입력 필드들
        input_widget = QWidget()
        input_layout = QVBoxLayout(input_widget)
        
        # Action 입력
        action_layout = QHBoxLayout()
        action_label = QLabel("Action:")
        self.action_input = QLineEdit()
        action_layout.addWidget(action_label)
        action_layout.addWidget(self.action_input)
        input_layout.addLayout(action_layout)
        
        # Mode 입력
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        self.mode_input = QLineEdit()
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_input)
        input_layout.addLayout(mode_layout)
        
        # Message 입력
        message_layout = QHBoxLayout()
        message_label = QLabel("Message:")
        self.message_input = QLineEdit()
        message_layout.addWidget(message_label)
        message_layout.addWidget(self.message_input)
        input_layout.addLayout(message_layout)
        
        layout.addWidget(input_widget)
        
        # 전송 버튼
        send_button = QPushButton("전송")
        send_button.setStyleSheet("""
            QPushButton {
                background-color: #5E77E1;
                border-radius: 10px;
                color: white;
                font-size: 18px;
                font-weight: bold;
                padding: 10px;
            }
        """)
        send_button.clicked.connect(self.send_message)
        layout.addWidget(send_button)
        
        # 수신 메시지 표시
        receive_label = QLabel("수신된 메시지:")
        layout.addWidget(receive_label)
        
        self.receive_text = QTextEdit()
        self.receive_text.setReadOnly(True)
        self.receive_text.setStyleSheet("background-color: white;")
        layout.addWidget(self.receive_text)
        
        # 비상정지 버튼
        emergency_button = QPushButton("비상정지")
        emergency_button.setStyleSheet("""
            QPushButton {
                background-color: #E23C3C;
                border-radius: 10px;
                color: white;
                font-size: 18px;
                font-weight: bold;
                padding: 10px;
            }
        """)
        emergency_button.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_button)
        
        self.setLayout(layout)
        self.resize(400, 600)
        
        # 메시지 수신 연결
        self.jetson.message_received.connect(self.handle_received_message)
    
    def send_message(self):
        try:
            action = self.action_input.text()
            mode = int(self.mode_input.text())
            message = self.message_input.text()
            
            asyncio.run_coroutine_threadsafe(
                self.jetson.send_data(action, mode, message),
                self.loop
            )
        except ValueError:
            self.receive_text.append("Error: Mode must be a number")
        except Exception as e:
            self.receive_text.append(f"Error: {str(e)}")
    
    def emergency_stop(self):
        try:
            asyncio.run_coroutine_threadsafe(
                self.jetson.emergency_stop(),
                self.loop
            )
            self.receive_text.append("Emergency stop signal sent")
        except Exception as e:
            self.receive_text.append(f"Emergency stop error: {str(e)}")
    
    def handle_received_message(self, message):
        self.receive_text.append(f"Received: {message}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = RobotUI()
    ui.show()
    
    # 비동기 초기화를 위한 별도 스레드
    async def init_async():
        await ui.init_jetson()
    
    # 새로운 이벤트 루프 생성 및 설정
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    ui.loop = loop
    
    # 백그라운드 태스크로 실행
    def run_async_loop():
        try:
            loop.run_until_complete(init_async())
            loop.run_forever()
        except Exception as e:
            print(f"Async loop error: {e}")
        finally:
            try:
                loop.stop()
                loop.close()
            except Exception as e:
                print(f"Loop cleanup error: {e}")
    
    # 별도 스레드에서 이벤트 루프 실행
    thread = threading.Thread(target=run_async_loop, daemon=True)
    thread.start()
    
    try:
        sys.exit(app.exec_())
    finally:
        ui.cleanup()
