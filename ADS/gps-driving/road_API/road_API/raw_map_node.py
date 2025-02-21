#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
import numpy as np
import requests
from PIL import Image as PILImage
from PIL import ImageDraw
import io
import math
import cv2
from cv_bridge import CvBridge
from concurrent.futures import ThreadPoolExecutor
import threading

class TileCache:
    def __init__(self, capacity):
        self.capacity = capacity
        self.cache = {}
        self.lock = threading.Lock()

    def get(self, key):
        with self.lock:
            return self.cache.get(key)

    def put(self, key, value):
        with self.lock:
            if len(self.cache) >= self.capacity:
                self.cache.pop(next(iter(self.cache)))
            self.cache[key] = value

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        
        # 초기화
        self.bridge = CvBridge()
        self.tile_cache = TileCache(50)
        self.thread_pool = ThreadPoolExecutor(max_workers=4)
        
        # 마우스 상호작용 변수
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.view_lat = None  # 현재 보고 있는 위치
        self.view_lon = None
        
        # 발행자 설정
        self.image_publisher = self.create_publisher(
            Image,
            'map_image',
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
        self.zoom = 17  # 초기 줌 레벨
        
        # 타일 설정
        self.tile_size = 256
        self.tiles_x = 2
        self.tiles_y = 2
        self.total_width = self.tile_size * self.tiles_x
        self.total_height = self.tile_size * self.tiles_y
        
        # OpenCV 창 생성 및 마우스 콜백 설정
        cv2.namedWindow('Map')
        cv2.setMouseCallback('Map', self.mouse_callback)
        
        # 지도 업데이트 타이머
        self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Map Visualizer has been started')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.drag_start_x = x
            self.drag_start_y = y
            
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            if self.view_lat is None:
                self.view_lat = self.current_lat
                self.view_lon = self.current_lon
                
            # 드래그 거리를 위도/경도 변화로 변환
            dx = x - self.drag_start_x
            dy = y - self.drag_start_y
            
            # 현재 줌 레벨에서의 위도/경도 변화량 계산
            lon_change = -dx * (360 / (256 * (2 ** self.zoom)))
            lat_change = dy * (170 / (256 * (2 ** self.zoom)))
            
            self.view_lat = self.view_lat + lat_change
            self.view_lon = self.view_lon + lon_change
            
            self.drag_start_x = x
            self.drag_start_y = y
            
            # 지도 업데이트
            self.update_map()
            
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            
        elif event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:  # 확대
                self.zoom = min(19, self.zoom + 1)
            else:  # 축소
                self.zoom = max(1, self.zoom - 1)
            self.update_map()

    def gps_callback(self, msg):
        if not (msg.latitude and msg.longitude):
            return
            
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
        
        if self.view_lat is None:
            self.view_lat = msg.latitude
            self.view_lon = msg.longitude

    def get_tile_info(self, lat, lon):
        n = 2.0 ** self.zoom
        xtile = int((lon + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
        return xtile, ytile

    def fetch_tile(self, url):
        cached_tile = self.tile_cache.get(url)
        if cached_tile is not None:
            return cached_tile.copy()  # 캐시된 이미지의 복사본 반환

        try:
            response = requests.get(url, headers={'User-Agent': 'Map_Viewer/1.0'}, timeout=5)
            response.raise_for_status()
            tile_img = PILImage.open(io.BytesIO(response.content))
            self.tile_cache.put(url, tile_img.copy())
            return tile_img
        except Exception as e:
            self.get_logger().warning(f'Error loading tile: {e}')
            return None

    def create_combined_map(self, center_lat, center_lon):
        center_xtile, center_ytile = self.get_tile_info(center_lat, center_lon)
        combined_img = PILImage.new('RGB', (self.total_width, self.total_height))
        
        tasks = []
        for y in range(self.tiles_y):
            for x in range(self.tiles_x):
                current_xtile = center_xtile + x - self.tiles_x // 2
                current_ytile = center_ytile + y - self.tiles_y // 2
                url = f"https://tile.openstreetmap.org/{self.zoom}/{current_xtile}/{current_ytile}.png"
                tasks.append((url, x, y))

        futures = []
        for url, _, _ in tasks:
            future = self.thread_pool.submit(self.fetch_tile, url)
            futures.append(future)

        for (url, x, y), future in zip(tasks, futures):
            try:
                tile = future.result()
                if tile is not None:
                    combined_img.paste(tile, (x * self.tile_size, y * self.tile_size))
                else:
                    combined_img.paste((200, 200, 200), 
                                     (x * self.tile_size, y * self.tile_size,
                                      (x+1) * self.tile_size, (y+1) * self.tile_size))
            except Exception as e:
                self.get_logger().warning(f'Error processing tile: {e}')
                combined_img.paste((200, 200, 200), 
                                 (x * self.tile_size, y * self.tile_size,
                                  (x+1) * self.tile_size, (y+1) * self.tile_size))

        return combined_img

    def get_pixel_position(self, lat, lon, center_lat, center_lon):
        center_xtile, center_ytile = self.get_tile_info(center_lat, center_lon)
        target_xtile, target_ytile = self.get_tile_info(lat, lon)
        
        n = 2.0 ** self.zoom
        x = ((lon + 180.0) / 360.0 * n - target_xtile) * self.tile_size
        y = ((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n - target_ytile) * self.tile_size
        
        pixel_x = int((target_xtile - center_xtile + self.tiles_x//2) * self.tile_size + x)
        pixel_y = int((target_ytile - center_ytile + self.tiles_y//2) * self.tile_size + y)
        
        return pixel_x, pixel_y

    def timer_callback(self):
        if self.current_lat is None or self.origin_lat is None:
            return
        self.update_map()

    def update_map(self):
        try:
            # 현재 보고 있는 위치나 GPS 위치 기준으로 지도 생성
            center_lat = self.view_lat if self.view_lat is not None else self.current_lat
            center_lon = self.view_lon if self.view_lon is not None else self.current_lon
            
            img = self.create_combined_map(center_lat, center_lon)
            
            # 현재 GPS 위치 표시
            if self.current_lat is not None and self.current_lon is not None:
                draw = ImageDraw.Draw(img)
                pixel_x, pixel_y = self.get_pixel_position(
                    self.current_lat, self.current_lon, center_lat, center_lon)
                
                # 지도 범위 내에 있을 때만 현재 위치 표시
                if 0 <= pixel_x < self.total_width and 0 <= pixel_y < self.total_height:
                    radius = 4
                    draw.ellipse((pixel_x - radius, pixel_y - radius, 
                                pixel_x + radius, pixel_y + radius), 
                                fill='red')
            
            # OpenCV 이미지로 변환
            img_array = np.array(img)
            img_cv = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            
            # 줌 레벨과 좌표 표시
            text = f'Zoom: {self.zoom}, Pos: {self.current_lat:.6f}, {self.current_lon:.6f}'
            cv2.putText(img_cv, text, (5, 15), cv2.FONT_HERSHEY_PLAIN, 
                       0.8, (0, 0, 255), 1)
            
            # OpenCV 창에 표시
            cv2.imshow('Map', img_cv)
            
            # ROS 이미지 메시지로 변환하여 발행
            img_msg = self.bridge.cv2_to_imgmsg(img_cv, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'map'
            self.image_publisher.publish(img_msg)
            
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                self.destroy_node()
                rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f'Error updating map: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.thread_pool.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()