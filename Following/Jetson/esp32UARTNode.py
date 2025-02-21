import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from time import sleep


class JetsonEsp32UARTNode(Node):
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyTHS0', baudrate=9600, timeout=1, bytesize=serial.EIGHTBITS)
        self.distance = -1
        self.angle = -1
        self.getDistanceAndAngle()

    def getDistanceAndAngle(self):
        i = 0

        while i < 100:
            if self.ser.in_waiting > 0:
                i += 1
                data = self.ser.readline(self.ser.in_waiting)
    
                decoded_data = data.decode('ascii', errors="ignore")
                print(decoded_data)
                try:
                    self.distance, self.angle = decoded_data.split()
                except Exception as e:
                    self.get_logger().error(f"Can not Split Data from ESP32: {e}")
                    continue

                print(f"distance: {self.distance}, angle: {self.angle}")




def main(args = None):
    rclpy.init(args=args)
    jetson_esp_node = JetsonEsp32UARTNode()
    rclpy.spin(jetson_esp_node)


if __name__ == '__main__':
    main()
