import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose
from robot_localization.srv import FromLL
from geometry_msgs.msg import PoseStamped


class YamlWaypointParser:
    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class GpsWpCommander(Node):
    def __init__(self, wps_file_path):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        
        # FromLL 서비스 클라이언트 생성
        self.localizer = self.create_client(FromLL, '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FromLL service...')
        
        self.client_futures = []
        self.get_logger().info('Ready to process waypoints')

    def convert_and_follow_waypoints(self):
        wps = self.wp_parser.get_wps()
        
        # GPS 좌표를 지도 좌표로 변환 요청
        for wp in wps:
            req = FromLL.Request()
            req.ll_point.longitude = wp.position.longitude
            req.ll_point.latitude = wp.position.latitude
            req.ll_point.altitude = wp.position.altitude
            
            self.get_logger().info(f"Converting waypoint: Lat={wp.position.latitude}, Lon={wp.position.longitude}")
            self.client_futures.append(self.localizer.call_async(req))

    def process_waypoints(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            
            if not self.client_futures:
                break
                
            incomplete_futures = []
            poses = []
            
            for future in self.client_futures:
                if future.done():
                    try:
                        result = future.result()
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.header.stamp = self.get_clock().now().to_msg()
                        pose.pose.position = result.map_point
                        poses.append(pose)
                        self.get_logger().info("Waypoint converted successfully")
                    except Exception as e:
                        self.get_logger().error(f'Service call failed: {str(e)}')
                else:
                    incomplete_futures.append(future)
            
            self.client_futures = incomplete_futures
            
            # 변환된 좌표가 있으면 네비게이션 실행
            if poses:
                for pose in poses:
                    self.get_logger().info("Navigating to converted waypoint...")
                    self.navigator.goToPose(pose)
                    
                    # 목표 도달 대기
                    while not self.navigator.isTaskComplete():
                        feedback = self.navigator.getFeedback()
                        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}')
                        time.sleep(0.1)

                    # 결과 확인은 단순히 task 완료 여부로 판단
                    if self.navigator.isTaskComplete():
                        self.get_logger().info('Waypoint reached successfully')
                    else:
                        self.get_logger().warning('Failed to reach waypoint')


def main():
    rclpy.init()

    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_gps_waypoint_follower_demo"), "config", "demo_waypoints.yaml")
    yaml_file_path = sys.argv[1] if len(sys.argv) > 1 else default_yaml_file_path

    try:
        gps_wpf = GpsWpCommander(yaml_file_path)
        gps_wpf.convert_and_follow_waypoints()
        gps_wpf.process_waypoints()
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()