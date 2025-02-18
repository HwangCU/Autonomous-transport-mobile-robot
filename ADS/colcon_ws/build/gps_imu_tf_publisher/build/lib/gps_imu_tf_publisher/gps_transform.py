import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
import tf2_ros
import pyproj

class GPSTransform(Node):
    def __init__(self):
        super().__init__('gps_transform_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)

        # WGS84 -> UTM 변환기
        self.utm_converter = pyproj.Transformer.from_crs("epsg:4326", "epsg:32652", always_xy=True)  # EPSG 번호는 지역에 맞게 수정

    def gps_callback(self, msg):
        utm_x, utm_y = self.utm_converter.transform(msg.longitude, msg.latitude)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"  # 변경됨: base_footprint 위치를 업데이트
        t.transform.translation.x = utm_x
        t.transform.translation.y = utm_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # GPS는 방향 정보를 제공하지 않음

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GPSTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
