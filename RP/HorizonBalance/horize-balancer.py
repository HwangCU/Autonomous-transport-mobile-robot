import SG90 as motor
import MPU6050 as sensor
from time import sleep

level_angle = motor.level_angle


A = 1.0
H = 1.0        # 상태 공간 방정식
Q = 0.1
R = 0.35       # 상태 공간 방정식 노이즈, 센서값 노이즈
x = 90
P = 1.0        # 이전 값, 오차 공분산

def kalman(value):
    global x, P

    xp = A * x
    pp = A * P * A + Q
    K = pp * H / (H * pp * H + R)
    x = xp + K*(value - H * xp)
    P = pp - K * H * pp

    return x
    

if __name__ == "__main__":
    # initialization
    sensor.MPU_Init()
    motor.startServo()

    x = sensor.readYaw('x')

    try:
        while True:
            angle = sensor.safe_readYaw('x')        # 센서는 - 회전방향, 서보모터는 + 회전방향

            # kalman
            angle = kalman(angle)

            dtheta = float(angle - 90)
            print(f"Theta: {dtheta:.1f} DEG", end=" / ")
            # dtheta가 음수일때는 모터 각도 보정 필요
            if dtheta > 0.0:
                dtheta *= 0.7
            else:
                dtheta *= 0.8

            # 센서 측정 값을 서보모터 제어 각도로 변환
            motor_degree = level_angle + dtheta
            

            motor.setServoPos(motor_degree)
            print(f"Motor: {motor_degree:.1f} DEG")
            sleep(0.18)                        # 무부하 상태 0.08 권장, 부하 상태 0.2 이상 권장

    except (KeyboardInterrupt, SystemExit):
        print("End balane proccess")
        motor.endServo()
    except OSError as err:
        print("Communication Error!", err)

    print("End balance proccess")
    motor.endServo()
