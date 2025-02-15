import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QDesktopWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtSvg import QSvgWidget

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # 동적으로 변경될 UI 요소
        self.unlock_button = None
        self.status_label = None
        self.caution_widget = None
        self.initUI()

        # 테스트 변수
        self.test_mode = ['standard', 'follow', 'follow-gohome', 'follow-home', 'delivery', 'delivery-stopover', 'delivery-arrived', 'error']
        self.test_index = 0
        
    def unlock_robot(self):
        print("로봇 잠금 해제")
        # 테스트 모드
        self.test_index += 1
        if self.test_index >= len(self.test_mode):
            self.test_index = 0
        self.update_ui_state(self.test_mode[self.test_index])
    
    def emergency_stop(self):
        print("비상 정지")
        # 테스트 모드
        self.test_index += 1
        if self.test_index >= len(self.test_mode):
            self.test_index = 0
        self.update_ui_state(self.test_mode[self.test_index])
        
    def update_ui_state(self, signal_value):
        if signal_value == 'standard': # 홈 화면
            self.status_label.setText("안녕하세요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal_value == 'follow': # 팔로잉 모드
            self.status_label.setText("팔로잉 모드가 실행 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal_value == 'follow-gohome': # 팔로잉 모드 - 집으로 돌아가기 중일 때
            self.status_label.setText("집으로 돌아가고 있어요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal_value == 'follow-home': # 팔로잉 모드 - 집에 도착했을 때
            self.status_label.setText("집에 도착했어요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal_value == 'delivery': # 배달 모드 - 배달 중
            self.status_label.setText("로봇이 배달 중이에요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal_value == 'delivery-stopover': # 배달 모드 - 주문한 가게 도착
            self.status_label.setText("로봇이 목적지에 도착했어요")
            self.unlock_button.show()
            self.caution_widget.show()
        elif signal_value == 'delivery-arrived': # 배달 모드 - 배달 완료
            self.status_label.setText("배달을 완료했어요")
            self.unlock_button.hide()
            self.caution_widget.show()
        elif signal_value == 'error': # 에러 상태
            self.status_label.setText("로봇에 문제가 발생했습니다")
            self.unlock_button.hide()
            self.caution_widget.hide()
        
    # UI 초기화 메서드
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
        
        star_icon = QSvgWidget("star_filled.svg")
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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    rb = RobotUI()
    rb.show()
    rb.update_ui_state('standard')  # 초기 상태 설정
    sys.exit(app.exec_())
