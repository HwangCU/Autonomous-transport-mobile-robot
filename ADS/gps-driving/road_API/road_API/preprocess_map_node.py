#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
import numpy as np
import requests
from PIL import Image
import io
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class EnhancedMapVisualizer(Node):
    def __init__(self):
        super().__init__('enhanced_map_visualizer')
        
        # TF 브로드캐스터 초기화
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 지도 발행자
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            'map',
            10)
            
        # 현재 위치 마커 발행자
        self.marker_publisher = self.create_publisher(
            Marker,
            'current_position',
            10)
            
        # GPS 구독
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
            
        self.current_lat = None
        self.current_lon = None
        self.origin_lat = None
        self.origin_lon = None
        self.zoom = 15  # 줌 레벨을 15로 낮춤
        
        # 지도 업데이트 타이머 (2초마다)
        self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Enhanced Map Visualizer has been started')

    def gps_callback(self, msg):
        if not (msg.latitude and msg.longitude):
            self.get_logger().warn('Received invalid GPS coordinates')
            return
            
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # 초기 원점 설정
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f'Origin set to: {self.origin_lat}, {self.origin_lon}')

    def timer_callback(self):
        if self.current_lat is None or self.origin_lat is None:
            return
            
        # TF 변환 발행
        self.publish_tf()
        # 지도 업데이트
        self.update_map()
        # 현재 위치 마커 발행
        self.publish_position_marker()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        x, y = self.gps_to_meters(self.current_lat, self.current_lon)
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def gps_to_meters(self, lat, lon):
        R = 6378137.0
        x = R * math.radians(lon - self.origin_lon) * math.cos(math.radians(self.origin_lat))
        y = R * math.radians(lat - self.origin_lat)
        return x, y

    def update_map(self):
        try:
            # 지도 타일 가져오기
            url = self.get_tile_url(self.current_lat, self.current_lon)
            
            headers = {
                'User-Agent': 'ROS2_Map_Viewer/1.0',
                'Accept': 'image/png',
            }
            
            response = requests.get(url, headers=headers, timeout=5)
            response.raise_for_status()
            
            # 이미지 처리
            img = Image.open(io.BytesIO(response.content))
            
            # RGB로 변환
            if img.mode != 'RGB':
                img = img.convert('RGB')
                
            img_array = np.array(img)
            
            # 이미지 크기 확인
            if img_array.shape[0] == 0 or img_array.shape[1] == 0:
                self.get_logger().error('Retrieved empty image')
                return
                
            self.get_logger().info(f'Image shape: {img_array.shape}')
            
            # OccupancyGrid 메시지 생성
            grid_msg = OccupancyGrid()
            grid_msg.header.frame_id = "map"
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            
            resolution = 1.0
            grid_msg.info.resolution = resolution
            grid_msg.info.width = img_array.shape[1]
            grid_msg.info.height = img_array.shape[0]
            
            # 원점 위치 설정
            x, y = self.gps_to_meters(self.current_lat, self.current_lon)
            grid_msg.info.origin.position.x = x - (grid_msg.info.width * resolution / 2)
            grid_msg.info.origin.position.y = y - (grid_msg.info.height * resolution / 2)
            grid_msg.info.origin.position.z = 0.0
            
            # 이미지를 그레이스케일로 변환하고 occupancy grid로 변환
            gray_img = np.mean(img_array, axis=2).astype(np.uint8)
            
            # OpenStreetMap 이미지에 맞게 변환 값 조정
            # 흰색 배경(255)은 0(free space)으로, 어두운 부분은 높은 값으로
            occupancy_data = np.zeros_like(gray_img, dtype=np.int8)
            
            # 도로와 같은 밝은 부분 (200-255)
            occupancy_data[gray_img > 200] = 0
            # 건물과 같은 어두운 부분 (0-100)
            occupancy_data[gray_img < 100] = 100
            # 중간 톤 (100-200)
            mid_tones = (gray_img >= 100) & (gray_img <= 200)
            occupancy_data[mid_tones] = 50
            
            grid_msg.data = occupancy_data.flatten().tolist()
            
            # 메타데이터 추가
            grid_msg.info.origin.orientation.w = 1.0
            
            self.map_publisher.publish(grid_msg)
            self.get_logger().info('Published map with size: {}x{}'.format(grid_msg.info.width, grid_msg.info.height))
            
        except Exception as e:
            self.get_logger().error(f'Error updating map: {str(e)}')
            self.get_logger().error(f'Error type: {type(e).__name__}')

    def get_tile_url(self, lat, lon):
        n = 2.0 ** self.zoom
        xtile = int((lon + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
        url = f"https://tile.openstreetmap.org/{self.zoom}/{xtile}/{ytile}.png"
        self.get_logger().info(f'Requesting tile: {url}')
        return url

    def publish_position_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        x, y = self.gps_to_meters(self.current_lat, self.current_lon)
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedMapVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()