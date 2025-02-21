from . import SG90 as motor
from . import MPU6050 as sensor
from time import sleep

class HorizonBalancer:
    def __init__(self):
        self.logger_prefix = "[HorizonBalancer]"
        
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
        try:
            angle = sensor.safe_readYaw('x')

            # kalman
            filtered_angle = self.kalman(angle)
            
            dtheta = float(filtered_angle - 90)
            
            if dtheta > 0.0:
                dtheta *= 0.7
            else:
                dtheta *= 0.8

            motor_degree = self.level_angle + dtheta
            
            if abs(motor_degree - self.pmotor_degree) > 2:
                motor.setServoPos(motor_degree)
                print(f"motor_degree: {motor_degree}")
                self.pmotor_degree = motor_degree
        except Exception as e:
            print(f"{self.logger_prefix} Error in balance_step: {e}")

    def cleanup(self) -> None:
        print(f"{self.logger_prefix} End balance process")
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
