import sys
import asyncio
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QDesktopWidget
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtSvg import QSvgWidget
from loadhorize import LoadHorize, LoadHorizeTest
from jetsoncommunication import JetsonCommunication
import threading
import json

# 비동기 작업을 처리하기 위한 스레드
class AsyncThread(QThread):
    signal = pyqtSignal(str)  # UI 업데이트를 위한 시그널

    def __init__(self, load_horize):
        super().__init__()
        self.load_horize = load_horize

    def run(self):
        asyncio.run(self.load_horize.main())

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.logger_prefix = "[RobotUI]"
        # 동적으로 변경될 UI 요소
        self.unlock_button = None
        self.status_label = None
        self.caution_widget = None
        
        # Jetson 연결 설정
        self.jetson = JetsonCommunication(server_ip='192.168.1.101')
        self.jetson.connection_status_changed.connect(self.handle_connection_status)
        
        # LoadHorize 인스턴스 생성
        # self.load_horize = LoadHorize()
        self.load_horize = LoadHorizeTest()
        
        # 비동기 작업을 위한 스레드 설정
        self.async_thread = AsyncThread(self.load_horize)
        self.async_thread.start()
        
        self.connection_retry_count = 0
        self.retry_interval = 3  # 재시도 간격 (초)

        # 기본 설정 메시지
        self.default_message = "standard"
        self.error_message = "error"
        self.connection_error_message = "connection_error"
        self.emergency_stop_message = "emergency_stop"
        
        self.loop = None  # loop를 None으로 초기화
        
        self.initUI()
        self.update_ui_state(self.default_message)
        
    # Jetson 연결 초기화
    async def init_jetson(self):
        if await self.jetson.connect():
            self.jetson.message_received.connect(self.handle_jetson_message)
    
    # Jetson으로부터 받은 메시지 처리
    def handle_jetson_message(self, message):
        self.update_ui_state(message)
    
    # 비상 정지 버튼 클릭 시 실행
    def emergency_stop(self):
        try:
            asyncio.run_coroutine_threadsafe(
                self.jetson.emergency_stop(), 
                self.loop
            )
            self.update_ui_state(self.error_message)
        except Exception as e:
            print(f"{self.logger_prefix} 비상정지 명령 처리 중 오류: {e}")

    # 로봇 잠금 해제 메시지 전송
    async def unlock_robot(self):
        await self.jetson.send_data('unlock_robot')

    def update_ui_state(self, signal):
        # 분기 처리
        if signal == 'standard': # 대기 모드
            self.status_label.setText("안녕하세요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'joycon': # 조이콘 모드
            self.status_label.setText("조이콘 모드가 실행 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'follow': # 팔로잉 모드
            self.status_label.setText("팔로잉 모드가 실행 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'gohome': # 집으로 돌아가기 중일 때
            self.status_label.setText("집으로 돌아가고 있어요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'home': # 집에 도착했을 때
            self.status_label.setText("집에 도착했어요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'delivery': # 배달 모드 - 배달 중
            self.status_label.setText("로봇이 배달 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'delivery-stopover': # 배달 모드 - 주문한 가게 도착
            self.status_label.setText("로봇이 목적지에 도착했어요")
            self.unlock_button.show()
            self.caution_widget.show()
        elif signal == 'delivery-arrived': # 배달 모드 - 배달 완료
            self.status_label.setText("배달을 완료했어요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal == 'error': # 에러 상태
            self.status_label.setText("로봇에 문제가 발생했습니다")
            self.unlock_button.hide()
            self.caution_widget.hide()
        elif signal == 'connection_error': # 젯슨과 통신 오류
            self.status_label.setText(f"로봇과 연결 중입니다... (시도 횟수: {self.connection_retry_count})")
            self.unlock_button.hide()
            self.caution_widget.hide()
        elif signal == 'emergency_stop': # 비상정지
            self.status_label.setText("비상정지 중 입니다")
            self.unlock_button.hide()
            self.caution_widget.hide()
        
    def handle_connection_status(self, is_connected):
        if is_connected:
            self.connection_retry_count = 0
            self.update_ui_state(self.default_message)
        else:
            self.connection_retry_count += 1
            print(f"{self.logger_prefix} 연결 재시도... (시도 횟수: {self.connection_retry_count})")
            self.update_ui_state(self.connection_error_message)

    def initUI(self):
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
        
        # 여백 설정
        main_layout.setContentsMargins(50, 50, 50, 50)

    # 창을 화면 중앙에 위치시키는 메서드
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def cleanup(self):
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    rb = RobotUI()
    rb.show()
    
    # 비동기 초기화를 위한 별도 스레드
    async def init_async():
        await rb.init_jetson()
    
    # 새로운 이벤트 루프 생성 및 설정
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    rb.loop = loop
    
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
        rb.cleanup()
