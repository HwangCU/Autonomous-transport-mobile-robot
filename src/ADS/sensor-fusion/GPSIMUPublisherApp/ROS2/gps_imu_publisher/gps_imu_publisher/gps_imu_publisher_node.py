#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from geometry_msgs.msg import TwistStamped
import websockets
import asyncio
import json
import threading
from transforms3d.euler import euler2quat
import numpy as np

class SensorPublisherNode(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # 퍼블리셔 생성
        self.gps_publisher = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_publisher = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.vel_publisher = self.create_publisher(TwistStamped, 'velocity', 10)
        
        # 웹소켓 서버 설정
        self.server = None
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    async def websocket_handler(self, websocket, path):
        self.get_logger().info("Client connected")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get('type', '')

                    if msg_type == 'gps':
                        self.handle_gps_data(data)
                    elif msg_type == 'imu':
                        self.handle_imu_data(data)
                    
                except Exception as e:
                    self.get_logger().error(f'Error processing message: {str(e)}')
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("Client disconnected")

    def handle_gps_data(self, data):
        # GPS 메시지 생성
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps"
        
        # GPS 데이터 설정
        gps_msg.latitude = float(data['latitude'])
        gps_msg.longitude = float(data['longitude'])
        gps_msg.altitude = float(data['altitude'])
        
        # 정확도 설정
        if 'accuracy' in data:
            accuracy = float(data['accuracy'])
            gps_msg.position_covariance[0] = accuracy ** 2
            gps_msg.position_covariance[4] = accuracy ** 2
            gps_msg.position_covariance[8] = accuracy ** 2
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # GPS 데이터 발행
        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f'Published GPS: Lat={gps_msg.latitude}, Lon={gps_msg.longitude}')

    def handle_imu_data(self, data):
        now = self.get_clock().now()
        
        # IMU 메시지 생성
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = "imu"

        # 가속도 데이터
        accel = data['accelerometer']
        imu_msg.linear_acceleration.x = float(accel['x'])
        imu_msg.linear_acceleration.y = float(accel['y'])
        imu_msg.linear_acceleration.z = float(accel['z'])

        # 자이로스코프 데이터
        gyro = data['gyroscope']
        imu_msg.angular_velocity.x = float(gyro['x'])
        imu_msg.angular_velocity.y = float(gyro['y'])
        imu_msg.angular_velocity.z = float(gyro['z'])

        # 자기장 데이터
        mag_msg = MagneticField()
        mag_msg.header.stamp = now.to_msg()
        mag_msg.header.frame_id = "imu"
        
        mag = data['magnetic']
        mag_msg.magnetic_field.x = float(mag['x'])
        mag_msg.magnetic_field.y = float(mag['y'])
        mag_msg.magnetic_field.z = float(mag['z'])

        # 방향과 속도 데이터
        calc = data['calculated']
        
        # 오일러 각을 쿼터니언으로 변환
        yaw = float(calc['yaw'])
        pitch = float(calc['pitch'])
        roll = float(calc['roll'])
        quat = euler2quat(yaw, pitch, roll)
        
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        # 속도 메시지
        vel_msg = TwistStamped()
        vel_msg.header.stamp = now.to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.twist.linear.x = float(calc['x_velocity'])
        vel_msg.twist.linear.y = float(calc['y_velocity'])
        vel_msg.twist.linear.z = float(calc['z_velocity'])

        # 모든 메시지 발행
        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)
        self.vel_publisher.publish(vel_msg)
        
        self.get_logger().info('Published IMU data')

    def run_server(self):
        async def serve():
            self.server = await websockets.serve(
                self.websocket_handler,
                "0.0.0.0",  # 모든 IP 접속 허용
                10310      # 포트 번호
            )
            self.get_logger().info("WebSocket server started on ws://0.0.0.0:10310")
            await self.server.wait_closed()

        asyncio.run(serve())

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()