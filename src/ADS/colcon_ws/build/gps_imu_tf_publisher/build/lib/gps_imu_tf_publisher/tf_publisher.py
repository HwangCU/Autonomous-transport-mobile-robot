import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class BaseFootprintToBaseLink(Node):
    def __init__(self):
        super().__init__('base_footprint_to_base_link_node')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # 변환 없음
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = BaseFootprintToBaseLink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
