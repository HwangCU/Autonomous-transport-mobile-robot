# controllers/pure_pursuit.py
import numpy as np
from math import atan2, sqrt, pi, sin, cos

class PurePursuit:
    def __init__(self, look_ahead_distance=5.0, wheel_base=2.0):
        self.wheel_base = 0.14  # 14cm를 미터 단위로 변환
        self.look_ahead_distance = 0.5  # 50cm로 축소 (차량 크기 고려)
        self.min_look_ahead = 0.3  # 최소 30cm
        self.max_look_ahead = 1.0  # 최대 1m

    def adjust_look_ahead(self, velocity):
        # 속도에 따른 look_ahead_distance 동적 조정
        look_ahead = velocity * 0.5 + self.min_look_ahead
        return np.clip(look_ahead, self.min_look_ahead, self.max_look_ahead)
    
    def calculate_steering_angle(self, current_pose, target_point, velocity=None):
        """Pure Pursuit 알고리즘으로 조향각 계산"""
        if velocity is not None:
            self.look_ahead_distance = self.adjust_look_ahead(velocity)
        # 현재 위치에서 목표점까지의 상대 위치 계산
        dx = target_point[0] - current_pose[0]
        dy = target_point[1] - current_pose[1]
        
        # 현재 heading을 고려한 좌표 변환
        current_heading = current_pose[2]
        target_x = dx * cos(current_heading) + dy * sin(current_heading)
        target_y = -dx * sin(current_heading) + dy * cos(current_heading)
        
        # Pure Pursuit 곡률 계산
        curvature = 2.0 * target_y / (self.look_ahead_distance ** 2)
        
        # 조향각 계산 (Ackermann steering)
        steering_angle = atan2(curvature * self.wheel_base, 1.0)
        
        return self.normalize_steering_angle(steering_angle)

    def normalize_steering_angle(self, angle):
        """조향각을 0~180도 범위로 정규화"""
        # 라디안을 도로 변환
        degrees = np.degrees(angle) + 90  # 중앙이 90도가 되도록 조정
        # 0~180도 범위로 제한
        return np.clip(degrees, 0, 180)