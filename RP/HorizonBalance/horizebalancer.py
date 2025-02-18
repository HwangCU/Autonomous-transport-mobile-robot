from . import SG90 as motor
from . import MPU6050 as sensor
from time import sleep

class HorizonBalancer:
    def __init__(self):
        # 칼만 필터 변수 초기화
        self.A = 1.0
        self.H = 1.0        # 상태 공간 방정식
        self.Q = 0.1
        self.R = 0.35       # 상태 공간 방정식 노이즈, 센서값 노이즈
        self.x = 90
        self.P = 1.0        # 이전 값, 오차 공분산
        
        # 초기화
        sensor.safe_MPU_Init()
        motor.startServo()
        self.x = sensor.safe_readYaw('x')
        self.level_angle = motor.level_angle
        self.pmotor_degree = self.level_angle

    def kalman(self, value: float) -> float:
        xp = self.A * self.x
        pp = self.A * self.P * self.A + self.Q
        K = pp * self.H / (self.H * pp * self.H + self.R)
        self.x = xp + K*(value - self.H * xp)
        self.P = pp - K * self.H * pp

        return self.x

    def balance_step(self) -> None:
        # 센서는 - 회전방향, 서보모터는 + 회전방향
        angle = sensor.safe_readYaw('x')

        # kalman
        angle = self.kalman(angle)

        dtheta = float(angle - 90)
        # print(f"Theta: {dtheta:.1f} DEG", end=" / ")
        
        # dtheta가 음수일때는 모터 각도 보정 필요
        if dtheta > 0.0:
            dtheta *= 0.7
        else:
            dtheta *= 0.8

        # 센서 측정 값을 서보모터 제어 각도로 변환
        motor_degree = self.level_angle + dtheta
        if abs(motor_degree - self.pmotor_degree) > 2:
            motor.setServoPos(motor_degree)
            self.pmotor_degree = motor_degree
            # print(f"Motor: {motor_degree:.1f} DEG")

    def cleanup(self) -> None:
        print("End balance process")
        motor.endServo()

# 테스트 코드
if __name__ == "__main__":
    balancer = HorizonBalancer()
    
    try:
        while True:
            balancer.balance_step()
            sleep(0.1)

    except (KeyboardInterrupt, SystemExit):
        balancer.cleanup()
    except OSError as err:
        print("Communication Error!", err)
        balancer.cleanup()
