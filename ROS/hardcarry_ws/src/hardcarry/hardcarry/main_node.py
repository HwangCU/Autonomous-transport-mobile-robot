import rclpy
from rclpy.node import Node
# from rclpy.action import ActionClient
from hardcarry_interface.msg import RobotMode
from hardcarry_interface.srv import DrivingControl, FollowingControl, JoyconControl


import time

# web socket

# import websocket
# import websocket
import json
# web_socket = None
# web_server_url = ""


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.get_logger().info(f"[HardCarry] Start initializing Main Node...")

        # websocket
        self.websocket = None
        self.websocket_uri = "c103.duckdns.org:9000"
        self.connect_websocket()

        # Robot Mode
        self.robot_mode = 0
        self.active_mode_client = None
        self._robot_mode_publisher = self.create_publisher(RobotMode, 'RobotMode', 10) # RobotMode topic publisher
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_robot_mode)
        self.get_logger().info("[HardCarry] RobotMode publisher created.")


        # Joycon Mode Service
        self._joycon_client = self.create_client(JoyconControl, '/hardcarry_joycon_service')
        self.get_logger().info("[HardCarry] JoyconControl client created.")


        # Following Mode Service
        self._following_client = self.create_client(FollowingControl, '/hardcarry_following_service')
        while not self._following_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("[HardCarry] FollowingControl client is not availavble, waiting again...")
            
        self.get_logger().info("[HardCarry] FollowingControl client created.")


        # Driving Mode Service - Delivery & Homming
        self._driving_client = self.create_client(DrivingControl, '/hardcarry_driving_service')
        self.get_logger().info("[HardCarry] DrivingControl client created.")





    def publish_robot_mode(self):
        # Publish robot mode topic
        msg = RobotMode()
        msg.mode = self.robot_mode
        self._robot_mode_publisher.publish(msg)
        self.get_logger().info(f"[HardCarry] Publish RobotMode: {self.robot_mode}.")

    def connect_websocket(self):
        # _robot_id = 


        url = f"wss://{self.websocket_uri}/socket/ws/ros/"

        #TODO 
        # access_token (get when login)
        # login -> post -> accesstoken -> socket open

        try:
            # self.websocket = websocket.WebSocketApp(
            #     url,
            #     on_message=self.main_control,
            #     on_error=self.websocket_error,
            #     on_close=self.websocket_close
            # )
            # self.websocket.on_open = self.websocket_open
            self.get_logger().info(f"[HardCarry] Websocket connection established.")
        except Exception as e:
            self.get_logger().error(f"[HardCarry] Error occured while connecting websocket: {e}.")

    def send_websocket(self, message):
        try:
            if self.websocket:
                #TODO need to modify 
                request_obj = {"message": message}
                json_parsed_obj = json.dumps(request_obj)
                self.websocket.send(json_parsed_obj)
                self.get_logger().info(f"[HardCarry] Sent message on webserver: {message}.")

        except Exception as e:
            self.get_logger().error(f"[HardCarry] Error occured while sending message({message}) using websocket: {e}.")
    
    def websocket_open(self,ws):
        self.get_logger().info(f"[HardCarry] Websocket open.")
        #TODO need to modify 
        openning_message_json = json.dump({"message": "opened"})
        self.websocket.send(openning_message_json)
    
    def websocket_error(self,ws,error):
        self.get_logger().error(f"[HardCarry] Error occured while receiving message from webserver {error}.")

    def websocket_close(self,ws, close_status_code, close_msg):
        self.get_logger().info(f"[HardCarry] Websocket Closed {close_status_code} {close_msg}.")
        
    def main_control(self,ws, message):
        # it will be called when webserver send a message
        try:
            #TODO need to modify
            # _socket_input = json.loads(message)
            _socket_input = message
            self.get_logger().info(f"[HardCarry] Received from server: {_socket_input}.")
            if _socket_input["action"] == "change_mode":
                input_robot_mode = _socket_input["mode"]
                if input_robot_mode == "0":
                    # Stop mode (Do nothing)
                    if self.active_mode_client:
                        if self.robot_mode == "1":
                            pass
                            
                        elif self.robot_mode == "2":
                            _request = FollowingControl.Request()
                            _request.enable_following = False
                            future = self._following_client.call_async(_request)
                            future.add_done_callback(self.service_req_deactivate_callback)
                        elif self.robot_mode == "3" or self.robot_mode == "4":
                            pass
                
                elif input_robot_mode == "1":
                    # Joycon
                    # print("Joycon")
                    self.get_logger().info(f"[HardCarry] Activating joycon mode.")
                    if self._request is not None:
                        #TODO other service actived. need to controlling error
                        # hadle case by active mode (is same or not)
                        pass
                    else:
                        _request = JoyconControl.Request()
                        _request.enable_joycon = True
                        future = self._joycon_client.call_async(_request)
                        self.robot_mode = input_robot_mode
                        future.add_done_callback(self.service_req_activate_callback)
                        self.active_mode_client = self._joycon_client

                elif input_robot_mode == "2":
                    # Following
                    # print("Following")
                    self.get_logger().info(f"[HardCarry] Activating following mode.")
                    _request = FollowingControl.Request()
                    _request.enable_following = True
                    future = self._following_client.call_async(_request)
                    self.robot_mode = input_robot_mode
                    future.add_done_callback(self.service_req_activate_callback)
                    self.active_mode_client = self._following_client

                elif input_robot_mode == "3" or input_robot_mode == "4":
                    if input_robot_mode == "3":
                        # Delivery
                        # print("Delivery")
                        self.get_logger().info(f"[HardCarry] Activating delivery mode.")
                        self.robot_mode = input_robot_mode

                    elif input_robot_mode == "4":
                        # Homming
                        # print("Homming")
                        self.get_logger().info(f"[HardCarry] Activating homming mode.")
                        self.robot_mode = input_robot_mode


                    _request = DrivingControl.Request()
                    _request.enable_driving = True
                    future = self._driving_client.call_async(_request)
                    future.add_done_callback(self.service_req_activate_callback)
                    self.active_mode_client = self._driving_client

            
        except Exception as e:
            self.get_logger().error(f"[HardCarry] Error occured: {e}.")
        finally:
            pass

    def service_req_activate_callback(self):
        # check if running service closed or not
        self.get_logger().info(f"[HardCarry] Mode activated.")
        
    def service_req_deactivate_callback(self):
        self.active_mode_client = None
        self.get_logger().info(f"[HardCarry] Mode deactivated.")

def main(args=None):
    try:
        rclpy.init(args=args)
        main_node = MainNode()
        # rclpy.spin(main_node)
        # main_node.connect_websocket()
        # time.sleep(1)
        # _sample_message = [{'action': 'change_mode', 'mode': '2'}, {'action': 'change_mode', 'mode': '0'},{'action': 'change_mode', 'mode': '2'},{'action': 'change_mode', 'mode': '0'}]
        _sample_message = [{'action': 'change_mode', 'mode': '2'}]
        
        for message in _sample_message:
            main_node.main_control(None, message)
            # time.sleep(5)
    except KeyboardInterrupt:
        main_node.main_control(None,{'action': 'change_mode', 'mode': '0'})

    except Exception as e:
        main_node.get_logger().error(f"[HardCarry] Error occured: {e}.")

    finally:
        main_node.get_logger().info(f"[HardCarry] Shutdown Main Node.")
        main_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


