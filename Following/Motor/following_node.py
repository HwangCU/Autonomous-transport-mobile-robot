import rclpy
from rclpy.node import Node

# UART
import serial

# Motor
import Jetson.GPIO as GPIO

# Time
import time

# Threading
# import threading

class FollowingNode:
    def __init__(self):
        # super().__init__('following_node')
        
        # pin number
        self.left_pwm1 = 33
        self.left_dir1 = 26
        self.right_pwm2 = 32
        self.right_dir2 = 23

        self.pwm = 0
        self.stable_pwm = 10
        self.control_left_pwm = 0
        self.control_right_pwm = 0
        self.forward = 0; #1: forward, 0: stop, -1: backward

        # left, right motor pwm
        self.left_pwm = None
        self.right_pwm = None
        
        self.left_prev_pwm = 0
        self.right_prev_pwm = 0

        # PID contorl
        self.p = 0.5
        
        
        # Serial communication
        self.ser = serial.Serial('/dev/ttyTHS0', baudrate=921600, timeout=1, bytesize=serial.EIGHTBITS)
        self.distance = -1
        self.angle = -1
        # self.get_logger().info(f"[HardCarry] following_node: Serial connection established.")
        

        # Motor
        self.motor_setup()
        self.get_logger().info(f"[HardCarry] following_node: Motor ready.")

        self.following()

            
        
    def motor_setup(self):
        GPIO.cleanup()
        
        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(self.left_dir1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_dir2, GPIO.OUT, initial=GPIO.LOW)

        GPIO.setup(self.left_pwm1, GPIO.OUT, initial=GPIO.HIGH)
        self.left_pwm = GPIO.PWM(self.left_pwm1, 10000)  # 1KHz
        self.left_pwm.start(0)

        time.sleep(0.1)
        GPIO.setup(self.right_pwm2, GPIO.OUT, initial=GPIO.HIGH)
        self.right_pwm = GPIO.PWM(self.right_pwm2, 10000)   # 1KHz
        self.right_pwm.start(0)
        
        # Move only Forward
        GPIO.output(self.left_dir1, GPIO.LOW)
        GPIO.output(self.right_dir2, GPIO.LOW)

    
    
    def following(self):
        print("follow node")
        try:
            # STOP = False
            while True:
                if self.ser.in_waiting > 0:
                    print("waiting...")
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
                        pass

                    self.get_logger().info(f"[HardCarry]  following_node: distance: {self.distance}, angle: {self.angle}")
                    

                    if  self.distance and self.distance > 1.2:
    
                        self.get_logger().info(f"[HardCarry] following_node: Move forward")
                        self.pwm = self.stable_pwm
                        self.control_left_pwm = self.stable_pwm
                        self.control_right_pwm = self.stable_pwm
                        
                            
                        if self.angle and self.angle >= 0 and self.angle <= 180:
                            temp_angle = int(self.angle / 10) * 10
                            # if temp_angle != self.angle :
                            self.angle = temp_angle
                            
                            if self.angle < 80:
                                # Turn right
                                self.get_logger().info(f"[HardCarry] following_node: Turn right")
                                # angle: 0 ... 90 -> stable_pwm: 0 ... stable_pwm => stable_pwm /90 *  self. angle
                                if self.control_right_pwm > (self.stable_pwm - ((self.stable_pwm / 90) * self.angle)): #always...
                                    self.control_right_pwm -= (self.stable_pwm - ((self.stable_pwm / 90) * self.angle))
                                
                                
                            elif self.angle > 110:
                                # Turn left
                                self.get_logger().info(f"[HardCarry] following_node: Turn left")
                                self.control_left_pwm -= (self.stable_pwm - ((self.stable_pwm / 90) * (self.angle-90)))
                                
                                pass
                            
                            else:
                                self.control_left_pwm = self.stable_pwm
                                self.control_left_pwm = self.stable_pwm
                    

                    else:
                        self.get_logger().info(f"[HardCarry] following_node: Stop")
                        self.control_left_pwm = 0
                        self.control_right_pwm = 0
                    
                    #here
                    
                    self.left_prev_pwm += int(self.control_left_pwm - self.left_prev_pwm) * self.p
                    self.right_prev_pwm += int(self.control_right_pwm - self.right_prev_pwm) * self.p
                    self.left_pwm.ChangeDutyCycle(self.left_prev_pwm)
                    self.right_pwm.ChangeDutyCycle(self.right_prev_pwm)
                    
                    self.get_logger().info(f"[HardCarry] following_node: left_control: {self.control_left_pwm}, right_control: {self.control_right_pwm}")
                    self.get_logger().info(f"[HardCarry] following_node: left_prev_control: {self.left_prev_pwm}, right_prev_control: {self.right_prev_pwm}")
                    
                    time.sleep(0.1)
                
                        
                    

            
        except Exception as e:
            self.get_logger().error(f"[HardCarry] following_node: Error occured: {e}.")
            pass
        finally:
            if self.right_pwm:
                self.right_pwm.stop()
            if self.left_pwm:
                self.right_pwm.stop()
            GPIO.cleanup()
            self.get_logger().info(f"[HardCarry] following_node: GPIO cleanup complete.")
            
            
def main(args=None):
    try:
        rclpy.init(args=args)
        following_node = FollowingNode()
        rclpy.spin(following_node)
        following_node.following()

    except Exception as e:
        following_node.get_logger().error(f"[HardCarry] following_node: Error occured: {e}.")
        pass
    finally:
        following_node.get_logger().info(f"[HardCarry] following_node: Finish Following Node.")
        following_node.destroy_node()
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()
