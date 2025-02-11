import rclpy
from rclpy.node import Node
import serial

from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import time
from time import sleep


class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60

    def set_throttle(self, throttle):
        pulse = int(0xFFFF * abs(throttle))

        if throttle > 0: # 전진
            self.pwm.channels[self.channel + 5].duty_cycle = pulse
            self.pwm.channels[self.channel + 4].duty_cycle = 0
            self.pwm.channels[self.channel + 3].duty_cycle = 0xFFFF

        elif throttle < 0: # 후진
            self.pwm.channels[self.channel + 5].duty_cycle = pulse
            self.pwm.channels[self.channel + 4].duty_cycle = 0xFFFF
            self.pwm.channels[self.channel + 3].duty_cycle = 0

        else: #정지
            self.pwm.channels[self.channel + 5].duty_cycle = 0
            self.pwm.channels[self.channel + 4].duty_cycle = 0
            self.pwm.channels[self.channel + 3].duty_cycle = 0


class FollowingNdoe(Node):
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 60 # PCA9685 주파수 설정

        # 모터 및 서보모터 초기화
        self.motor_hat = PWMThrottleHat(self.pca, channel=0)
        self.kit = ServoKit(channels=16, i2c=self.i2c, address=0x60)
        self.pan = 90 # 서보모터 초기 위치 설정
        self.kit.servo[0].angle = self.pan

        self.ser = serial.Serial('/dev/ttyTHS0', baudrate=921600, timeout=1, bytesize=serial.EIGHTBITS)
        self.distance = -1
        self.angle = -1

        self.getDistanceAndAngle()

    
    def getDistanceAndAngle(self):
        try:
            # i = 0
            STOP = False
            while True:
                if self.ser.in_waiting > 0:
                    # i += 1
                    data = self.ser.readline(self.ser.in_waiting)

                    decoded_data = data.decode('ascii', errors="ignore")
                    print(decoded_data)
                    try:
                        datas = [data if "\\x" not in data else 'nan' for data in decoded_data.split()]
                        print(datas)
                        if('nan' not in datas and len(datas) == 2):
                            self.distance, self.angle = [float(data) for data in datas]
                    except Exception as e:
                        self.get_logger().error(f"Can not Split Data from ESP32: {e}")
                        continue

                    print(f"distance: {self.distance}, angle: {self.angle}")
                    


                    if STOP:
                        self.motor_hat.set_throttle(0)
                    

                    else:
                        if self.angle and self.angle >= 0 and self.angle <= 180:
                            temp_angle = int(self.angle / 10) * 10;
                            # if temp_angle != self.angle :
                            self.angle = temp_angle;

                            self.pan = 180 - self.angle

                            self.kit.servo[0].angle = 180 - self.angle
                            print("Servo Angle: ", self.pan)


                        else:

                            self.pan = 100 
                            self.kit.servo[0].angle = self.pan    

                        if  self.distance and self.distance > 1.0:
        
                            print("Motor Forward")
                            self.motor_hat.set_throttle(0.5)
                        else:
                            print("Stop")
                            self.motor_hat.set_throttle(0)

        except KeyboardInterrupt:
            print(f"KeyboardInterrupt")

        finally:
            self.motor_hat.set_throttle(0) # 모터 정지
            self.kit.servo[0].angle = 100 # 서보모터 초기 위치로 이동
            self.pca.deinit() # PCA9685 종료  


def main(args = None):

    try:
        rclpy.init(args=args)
        following_node = FollowingNdoe()
        rclpy.spin(following_node)
    except Exception as e:
        following_node.get_logger().error(f"Exception occured: {e}")
    
    finally:
        print("Program Exit")

if __name__ == '__main__':
    main()

