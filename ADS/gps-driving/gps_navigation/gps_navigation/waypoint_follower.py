#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from custom_interfaces.msg import AckermannDrive
import numpy as np
import csv
import math
from gps_navigation.utils.coordinate_utils import CoordinateConverter
from gps_navigation.controllers.pure_pursuit import PurePursuit

class WaypointFollower(Node):
   def __init__(self):
       super().__init__('waypoint_follower')
       
       # 파라미터
       self.declare_parameter('waypoints_file', 'config/waypoints.csv')
       self.declare_parameter('goal_tolerance', 0.2)  # 미터 단위
       waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
       self.get_logger().info(f"Waypoints file: {waypoints_file}")

       # 초기화
       self.coord_converter = CoordinateConverter()
       self.pure_pursuit = PurePursuit()
       self.current_pose = None
       self.current_heading = None
       self.current_velocity = 0.0
       
       # 웨이포인트 로드
       self.waypoints = self.load_waypoints()
       self.current_waypoint_index = 0

       # Subscribers
       self.gps_sub = self.create_subscription(
           NavSatFix,
           'gps/fix',
           self.gps_callback,
           10)
           
       self.imu_sub = self.create_subscription(
           Imu,
           'imu/data',
           self.imu_callback,
           10)
           
       # Publishers
       self.control_pub = self.create_publisher(
           AckermannDrive,
           'cmd_ackermann',
           10)

       # 상태 Publishers
       self.distance_pub = self.create_publisher(
           Float64,
           'distance_to_waypoint',
           10)
       
       # 제어 타이머
       self.control_timer = self.create_timer(0.1, self.control_loop)
       
       self.get_logger().info('Waypoint follower initialized')
       
   def load_waypoints(self):
       waypoints = []
       file_path = self.get_parameter('waypoints_file').value
       try:
           with open(file_path, 'r') as file:
               reader = csv.DictReader(file)
               for row in reader:
                   waypoints.append({
                       'lat': float(row['latitude']),
                       'lon': float(row['longitude']),
                       'speed': float(row['speed'])
                   })
           self.get_logger().info(f'Loaded {len(waypoints)} waypoints')
           return waypoints
       except Exception as e:
           self.get_logger().error(f'Failed to load waypoints: {str(e)}')
           return []

   def imu_callback(self, msg):
       """IMU 데이터로 현재 방향 업데이트"""
       orientation = msg.orientation
       _, _, self.current_heading = self.quaternion_to_euler(
           orientation.w, orientation.x, orientation.y, orientation.z)
        
        # Calculate current velocity from linear acceleration
       linear_accel = msg.linear_acceleration
       dt = 0.1  # Time step (assuming 10Hz update rate)
       self.current_velocity += linear_accel.x * dt

   def gps_callback(self, msg):
       """GPS 데이터로 현재 위치 업데이트"""
       self.current_pose = [
           msg.latitude,
           msg.longitude,
           self.current_heading if self.current_heading is not None else 0.0
       ]

   def control_loop(self):
       """주 제어 루프"""
       if self.current_pose is None:
           self.get_logger().warn('No GPS data received')
           # 정지 명령 전송
           self.publish_control(0.0, 90.0)  # 중립 위치로 조향
           return

       if self.current_waypoint_index >= len(self.waypoints):
           self.get_logger().info('All waypoints reached!')
           # 정지 명령 전송
           self.publish_control(0.0, 90.0)  # 중립 위치로 조향
           return

       # 현재 목표 웨이포인트
       target = self.waypoints[self.current_waypoint_index]
       
       # 현재 위치와 목표점 사이의 거리 계산
       distance = self.coord_converter.calculate_distance(
           self.current_pose[0], self.current_pose[1],
           target['lat'], target['lon']
       )

       # 거리 정보 발행
       distance_msg = Float64()
       distance_msg.data = distance
       self.distance_pub.publish(distance_msg)
       self.get_logger().info(f'distance: {distance}m')

       # 목표점에 도달했는지 확인
       if distance < self.get_parameter('goal_tolerance').value:
           self.current_waypoint_index += 1
           self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
           return

       # UTM 좌표로 변환
       current_utm = self.coord_converter.geodetic_to_utm(
           self.current_pose[0], self.current_pose[1])
       target_utm = self.coord_converter.geodetic_to_utm(
           target['lat'], target['lon'])

       # 조향각 계산
       steering = self.pure_pursuit.calculate_steering_angle(
           [current_utm[0], current_utm[1], self.current_heading],
           [target_utm[0], target_utm[1]],
           self.current_velocity
       )

       # 속도 계산 (목표점까지의 거리에 따라 조정)
       throttle = min(target['speed'], 
                     self.calculate_adaptive_speed(distance))

       # 제어 명령 발행
       self.publish_control(throttle, steering)

   def publish_control(self, throttle, steering):
       """제어 명령 발행"""
       control_msg = AckermannDrive()
       control_msg.throttle = float(throttle)
       control_msg.steering = float(steering)
       self.control_pub.publish(control_msg)
       
   def calculate_adaptive_speed(self, distance):
       """거리에 따른 적응형 속도 계산"""
       min_speed = 0.2
       max_speed = 1.0
       ramp_distance = 10.0  # 미터
       
       # 거리에 따른 선형 속도 계산
       speed = (distance / ramp_distance) * max_speed
       return np.clip(speed, min_speed, max_speed)

   @staticmethod
   def quaternion_to_euler(w, x, y, z):
       """쿼터니언을 오일러 각으로 변환"""
       # Roll (x-axis rotation)
       sinr_cosp = 2 * (w * x + y * z)
       cosr_cosp = 1 - 2 * (x * x + y * y)
       roll = math.atan2(sinr_cosp, cosr_cosp)

       # Pitch (y-axis rotation)
       sinp = 2 * (w * y - z * x)
       pitch = math.asin(sinp)

       # Yaw (z-axis rotation)
       siny_cosp = 2 * (w * z + x * y)
       cosy_cosp = 1 - 2 * (y * y + z * z)
       yaw = math.atan2(siny_cosp, cosy_cosp)

       return roll, pitch, yaw

   def on_shutdown(self):
       """노드 종료 시 처리"""
       self.get_logger().info('Shutting down waypoint follower')
       # 정지 명령 전송
       self.publish_control(0.0, 90.0)

def main(args=None):
   rclpy.init(args=args)
   node = WaypointFollower()
   
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.on_shutdown()
       node.destroy_node()
       rclpy.shutdown()

if __name__ == '__main__':
   main()