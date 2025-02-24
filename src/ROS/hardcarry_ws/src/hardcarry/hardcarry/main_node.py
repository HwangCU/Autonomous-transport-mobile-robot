import rclpy
from rclpy.node import Node
# from rclpy.action import ActionClient
from std_msgs.msg import Int16
from hardcarry_interface.srv import DrivingControl, FollowingControl, JoyconControl, EmergencyStop

# Launch파일을 실행을 위한 import
import subprocess

import time
import json


class MainNode(Node):
    def __init__(self):
        # mode
        # mode : 0   대기 모드
        # mode : 1   조이콘 모드
        # mode : 2   팔로잉 모드
        # mode : 4   집으로 돌아가기 중일 때
        # mode : 41  집에 도착했을 때
        # mode : 3   배달 모드 - 배달 중
        # mode : 31  배달 모드 - 주문한 가게 도착
        # mode : 32  배달 모드 - 배달 완료
        # mode : 100 에러 상태
        # mode : 127 테스트 모드
        self.mode_kind = ["Stop_Mode", "Joy-Stick_Mode", "Following_Mode","Delivery_Mode","Homing_Mode"]
        super().__init__('main_node')
        self.get_logger().info(f"[HardCarry] Start initializing Main Node...")
        
        self.current_robot_mode = 0
        self.pre_robot_mode = 0
        # bringup process 변수 초기화 추가
        self._bringup_process = None
        
        # robot_mode subscribe        
        self.subscription = self.create_subscription(
            Int16,
            '/robot_mode',
            self.main_control,
            10
        )
        
        # Joycon Mode Service
        self._joycon_client = self.create_client(JoyconControl, '/hardcarry_joycon_service')
        # while not self._joycon_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("[HardCarry] JoyconControl client is not availavble, waiting again...")
        
        # self.get_logger().info("[HardCarry] JoyconControl client created.")

        # Following Mode Service
        self._following_client = self.create_client(FollowingControl, '/hardcarry_following_service')
        # while not self._following_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("[HardCarry] FollowingControl client is not availavble, waiting again...")
            
        # self.get_logger().info("[HardCarry] FollowingControl client created.")
        
        # Emergency Stop Service Server
        try:
            self.emergency_stop_service = self.create_service(
                EmergencyStop,
                'emergency_stop',
                self.emergency_stop_callback
            )
            self.get_logger().info("[HardCarry] Emergency Stop service server created successfully")
        except Exception as e:
            self.get_logger().error(f"[HardCarry] Failed to create Emergency Stop service server: {str(e)}")
        
    # 정지 상태 함수
    def stop_mode(self, pre_mode):
        if pre_mode == "1":
            _request = JoyconControl.Request()
            _request.enable_joycon = False
            future = self._joycon_client.call_async(_request)
            future.add_done_callback(self.service_req_deactivate_callback)
        elif pre_mode == "2":
            _request = FollowingControl.Request()
            _request.enable_following = False
            future = self._following_client.call_async(_request)
            future.add_done_callback(self.service_req_deactivate_callback)
        elif pre_mode == "3" or pre_mode == "4":
            self.get_logger().info(f"[HardCarry] _bringup_process is terminated")
            self._bringup_process.terminate()
            try:
                self._bringup_process.wait(timeout=5)  # 5초 동안 종료 대기
            except subprocess.TimeoutExpired:
                self._bringup_process.kill()  # 강제 종료
                
                
    # joycontrol 모드
    def joycon_mode(self):
        self.get_logger().info(f"[HardCarry] Activating joycon mode.")
        _request = JoyconControl.Request()
        _request.enable_joycon = True
        future = self._joycon_client.call_async(_request)
        future.add_done_callback(self.service_req_activate_callback)
    
    def following_mode(self):
        self.get_logger().info(f"[HardCarry] Activating following mode.")
        _request = FollowingControl.Request()
        _request.enable_following = True
        future = self._following_client.call_async(_request)
        future.add_done_callback(self.service_req_activate_callback)
        
    def delivery_mode(self):
        pass
    def homing_mode(self):
        pass
    
    # 메시지를 받았을 때 실행되는 콜백 함수
    def main_control(self, robot_mode_msg):
        # it will be called when webserver send a message
        try:
            #TODO need to modify
            self.current_robot_mode = robot_mode_msg.data
            # self.get_logger().info(f'Current robot mode: {self.current_robot_mode}')
            
            if self.current_robot_mode == 0:
                # Stop mode (Do nothing)
                if self.pre_robot_mode != 0:
                    self.get_logger().info(f'Current robot mode: {self.mode_kind[self.current_robot_mode]}')
                    self.stop_mode(self.current_robot_mode)
                self.pre_robot_mode = 0
                
            elif self.current_robot_mode == 1:
                # Joycon
                # print("Joycon")
                if self.pre_robot_mode != 1:
                    self.get_logger().info(f'Current robot mode: {self.mode_kind[self.current_robot_mode]}')
                    self.joycon_mode()
                self.pre_robot_mode = 1
                
            elif self.current_robot_mode == 2:
                # Following
                # print("Following")
                if self.pre_robot_mode != 2:
                    self.get_logger().info(f'Current robot mode: {self.mode_kind[self.current_robot_mode]}')
                    self.following_mode()
                self.pre_robot_mode = 2

            elif self.current_robot_mode == 3:
                if self.pre_robot_mode != 3 and self.pre_robot_mode != 4:
                    self.get_logger().info(f'Current robot mode: {self.mode_kind[self.current_robot_mode]}')
                    # bring up
                    try:
                        self._bringup_process = subprocess.Popen(
                            ["ros2", "launch", "stella_bringup", "robot.launch.py"],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE
                        )
                        stdout, stderr = self._bringup_process.communicate()
                    except Exception as e:
                        self.get_logger().error(f'Error: {e}')
                    
                self.pre_robot_mode = 3

            elif self.current_robot_mode == 4:
                if self.pre_robot_mode != 3 and self.pre_robot_mode != 4:
                    self.get_logger().info(f'Current robot mode: {self.mode_kind[self.current_robot_mode]}')
                    # bring up
                    try:
                        self._bringup_process = subprocess.Popen(
                            ["ros2", "launch", "stella_bringup", "robot.launch.py"])
                    except Exception as e:
                        self.get_logger().error(f'Error: {e}')
                self.pre_robot_mode = 4
            else:
                self.get_logger().info(f"[HardCarry] recieved wrong mode...{self.current_robot_mode}")
                

        except Exception as e:
            self.get_logger().error(f"[HardCarry] Error occured: {e}.")
        finally:
            # bringup process가 존재할 때만 종료
            if self._bringup_process is not None:
                self._bringup_process.terminate()
            pass

    def service_req_activate_callback(self):
        # check if running service closed or not
        self.get_logger().info(f"[HardCarry] Mode activated.")
        
    def service_req_deactivate_callback(self):
        self.active_mode_client = None
        self.get_logger().info(f"[HardCarry] Mode deactivated.")

    def emergency_stop_callback(self, request, response):
        try:
            if request.emergency_stop:
                self.get_logger().warn("[HardCarry] Emergency Stop Requested!")
                # 여기에 긴급 정지를 위한 추가 로직 구현
                
                response.success = True
                response.message = "Emergency stop executed successfully"
                self.get_logger().info("[HardCarry] Emergency stop completed")
            else:
                response.success = False
                response.message = "Invalid emergency stop request"
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop failed: {str(e)}"
            self.get_logger().error(f"[HardCarry] Emergency stop error: {str(e)}")
        
        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        main_node = MainNode()
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        main_node.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        main_node.get_logger().error(f"[HardCarry] Error occured: {e}.")

    finally:
        main_node.get_logger().info(f"[HardCarry] Shutdown Main Node.")
        main_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


