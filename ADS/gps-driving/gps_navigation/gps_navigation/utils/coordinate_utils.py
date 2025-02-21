# utils/coordinate_utils.py
# 좌표 변환 및 거리 계산 유틸리티:
import pyproj
from math import radians, sin, cos, sqrt, atan2

class CoordinateConverter:
    def __init__(self):
        self.proj_wgs84 = pyproj.CRS('EPSG:4326')  # WGS84 (GPS)
        self.proj_utm = pyproj.CRS('EPSG:32652')   # UTM52N (한국 기준)
        self.transformer = pyproj.Transformer.from_crs(
            self.proj_wgs84,
            self.proj_utm,
            always_xy=True
        )

    def geodetic_to_utm(self, lat, lon):
        """GPS 좌표를 UTM 좌표로 변환"""
        easting, northing = self.transformer.transform(lon, lat)
        return easting, northing

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """두 GPS 좌표 사이의 거리 계산 (meters)"""
        R = 6371000  # 지구 반경 (meters)
        
        lat1, lon1 = radians(lat1), radians(lon1)
        lat2, lon2 = radians(lat2), radians(lon2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """두 GPS 좌표 사이의 방위각 계산 (radians)"""
        lat1, lon1 = radians(lat1), radians(lon1)
        lat2, lon2 = radians(lat2), radians(lon2)
        
        dlon = lon2 - lon1
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        
        return atan2(y, x)