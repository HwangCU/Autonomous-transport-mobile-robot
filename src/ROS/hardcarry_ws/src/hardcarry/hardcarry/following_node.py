import rclpy
from rclpy.node import Node
# Service
from hardcarry_interface.srv import FollowingControl

# UART
import serial

# Motor
import Jetson.GPIO as GPIO

# Time
import time

# Threading
import threading

class FollowingNode(Node):
    def __init__(self):
        super().__init__('following_node')
        
        # pin number
        self.left_pwm1 = 33
        self.left_dir1 = 26
        self.right_pwm2 = 32
        self.right_dir2 = 23

        self.pwm = 0
        self.stable_pwm = 55
        self.control_left_pwm = 0
        self.control_right_pwm = 0
        self.forward = 0; #1: forward, 0: stop, -1: backward

        # left, right motor pwm
        self.left_pwm = None
        self.right_pwm = None
        
        self.left_prev_pwm = 0
        self.right_prev_pwm = 0

        # PID contorl
        self.p = 0.3
        
        
        # Serial communication
        self.ser = serial.Serial('/dev/ttyTHS0', baudrate=921600, timeout=1, bytesize=serial.EIGHTBITS)
        self.distance = -1
        self.angle = -1
        self.get_logger().info(f"[HardCarry] following_node: Serial connection established.")
        

        # Motor
        self.motor_setup()
        self.get_logger().info(f"[HardCarry] following_node: Motor ready.")

        # program control
        self._following_activate = False
        self._following_thread = threading.Thread(target=self.following)
        self._following_stop = threading.Event()

        # Service
        self._following_service = self.create_service(FollowingControl, '/hardcarry_following_service', self.following_callback)
        self.get_logger().info(f"[HardCarry] following_node: FollowingControl service server established.")
        self.get_logger().info(f"[HardCarry] following_node: FollowingControl service server waiting request...")
        

        
    
    def following_callback(self, request, response):
        self.get_logger().info(f"[HardCarry] following_node: FollowingControl service server received request: {request}.")

        if request.enable_following == True:
            if self._following_activate == False:
                #TODO start thread
                self._following_activate = True
                self._following_thread.start()
                
                response.message = "Following mode is successfully activated"
                self.get_logger().info(f"[HardCarry] following_node: Following mode is successfully activated.")

            else:
                response.success = False
                response.message = f"Following moode already running"
                self.get_logger().error(f"[HardCarry] following_node: Following moode already running.")
                
                return response
        elif request.enable_following == False:
            if self._following_activate == True:
                #TODO quit following
                self._following_stop.set()
                self._following_thread.join()
                self._following_thread = threading.Thread(target=self.following)
                self._following_stop.clear()
                self._following_activate = False
                response.message = "Following mode is successfully deactivated"
                self.get_logger().info(f"[HardCarry] following_node: Following mode is successfully deactivated.")
                
            else:
                response.success = False
                response.message = f"Following moode is not working."
                self.get_logger().error(f"[HardCarry] following_node: Following moode is not working.")
                
                return response
        else:
            response.success = False
            response.message = f"Invalid service request: {request}."
            self.get_logger().error(f"Invalid service request: {request}.")
            return response
        
        response.success = True

        return response

    
            
        
    def motor_setup(self):
        # 기존의 GPIO 설정을 초기화
        GPIO.cleanup()
        
        # GPIO 모드 설정
        GPIO.setmode(GPIO.BOARD)
        
        # 핀 설정
        GPIO.setup(self.left_dir1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_dir2, GPIO.OUT, initial=GPIO.LOW)

        # PWM 객체 생성 - 다른 주파수 사용
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
        self.get_logger().info(f"[HardCarry] following_node thread: Following mode start.")
        try:
            # STOP = False
            # while self._following_activate and not self._following_stop.is_set():
            while not self._following_stop.is_set():
                # self.get_logger().info(f"[HardCarry] following_node thread: following_stop: {self._following_stop.is_set()}.")
                
                # self.get_logger().info(f"[HardCarry] following_node thread: Waiting UWB datas...")
                
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

                    self.get_logger().info(f"[HardCarry]  following_node thread: distance: {self.distance}, angle: {self.angle}.")
                    

                    # if STOP:
                    #     self.motor_hat.set_throttle(0)
                    

                    # else:
                    if  self.distance and self.distance > 2.0:
    
                        self.get_logger().info(f"[HardCarry] following_node thread: Move forward.")
                        self.pwm = self.stable_pwm
                        self.control_left_pwm = self.stable_pwm
                        self.control_right_pwm = self.stable_pwm
                        
                            
                        if self.angle and self.angle >= 0 and self.angle <= 180:
                            temp_angle = int(self.angle / 10) * 10
                            # if temp_angle != self.angle :
                            self.angle = temp_angle
                            
                            if self.angle < 75:
                                # Turn right
                                self.get_logger().info(f"[HardCarry] following_node thread: Turn right.")
                                # angle: 0 ... 90 -> stable_pwm: 0 ... stable_pwm => stable_pwm /90 *  self. angle
                                # if self.control_right_pwm > (self.stable_pwm - ((self.stable_pwm / 90) * self.angle)): #always...
                                # self.control_right_pwm -= (self.stable_pwm - ((self.stable_pwm / 90) *0.5* self.angle))
                                self.control_right_pwm = (self.stable_pwm - 15)
                                
                                
                                
                            elif self.angle > 105:
                                # Turn left
                                self.get_logger().info(f"[HardCarry] following_node thread: Turn left.")
                                # self.control_left_pwm -= (self.stable_pwm - ((self.stable_pwm / 90) *0.5* (self.angle-90)))
                                self.control_left_pwm = (self.stable_pwm - 15)
                                
                                pass
                            
                            else:
                                self.control_left_pwm = self.stable_pwm
                                self.control_right_pwm = self.stable_pwm
                    

                    else:
                        self.get_logger().info(f"[HardCarry] following_node thread: Stop")
                        self.control_left_pwm = 0
                        self.control_right_pwm = 0
                    
                    #here
                    
                    self.left_prev_pwm += int(self.control_left_pwm - self.left_prev_pwm) * self.p
                    self.right_prev_pwm += int(self.control_right_pwm - self.right_prev_pwm) * self.p
                    self.left_pwm.ChangeDutyCycle(self.left_prev_pwm)
                    self.right_pwm.ChangeDutyCycle(self.right_prev_pwm)
                    
                    self.get_logger().info(f"[HardCarry] following_node: left_control: {self.control_left_pwm}, right_control: {self.control_right_pwm}")
                    self.get_logger().info(f"[HardCarry] following_node: left_prev_control: {self.left_prev_pwm}, right_prev_control: {self.right_prev_pwm}")
                    
                    time.sleep(0.01)
            self.get_logger().info(f"[HardCarry] following_node: Following mode stop.")
                
                        
                    

            
        except Exception as e:
            self.get_logger().error(f"[HardCarry] following_node: Error occured: {e}.")
            
        finally:
            pass
            
            
def main(args=None):
    try:
        rclpy.init(args=args)
        following_node = FollowingNode()
        rclpy.spin(following_node)
        # following_node._following_thread.start()

    except Exception as e:
        following_node.get_logger().error(f"[HardCarry] following_node: Error occured: {e}.")

    finally:
        if following_node.right_pwm:
                following_node.right_pwm.stop()
        if following_node.left_pwm:
            following_node.right_pwm.stop()
        GPIO.cleanup()
        following_node.get_logger().info(f"[HardCarry] following_node: GPIO cleanup complete.")
        if following_node._following_thread.isAlive:
            following_node._following_stop.set()
        following_node.get_logger().info(f"[HardCarry] following_node: Finish Following Node.")
        following_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()