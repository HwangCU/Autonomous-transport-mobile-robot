import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped

class IMUToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom_node')
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.imu_sub = self.create_subscription(Float64, "/imu/yaw", self.imu_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def imu_callback(self, msg):
        self.th = math.radians(msg.data)

        # 이동 거리 추정 (IMU 데이터 기반)
        dt = 0.1  # 10Hz 주기로 업데이트 (가정)
        vx = 0.1  # 예제: 전진 속도 (m/s) (이 값을 다른 센서에서 받아와야 함)
        vy = 0.0  # IMU 기반 오도메트리는 보통 y 이동 없음
        
        self.x += vx * math.cos(self.th) * dt
        self.y += vx * math.sin(self.th) * dt

        time_now = self.get_clock().now().to_msg()

        # odom -> base_footprint 변환
        t = TransformStamped()
        t.header.stamp = time_now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # 오도메트리 메시지 발행
        odom = Odometry()
        odom.header.stamp = time_now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = IMUToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
