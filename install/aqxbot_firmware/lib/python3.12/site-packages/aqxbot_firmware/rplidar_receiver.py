import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json
import math
import time

class LidarReceiver(Node):
    def __init__(self):
        super().__init__('lidar_udp_receiver')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8101))
        self.sock.settimeout(0.1)  # Non-blocking

        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(1)
        self.range_max = 4.0

    def timer_callback(self):
        try:
            data, _ = self.sock.recvfrom(65536)
            scan_data = json.loads(data)

            num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
            ranges = [float('inf')] * num_readings

            for point in scan_data:
                angle_deg = point['angle']
                distance_mm = point['distance']
                if distance_mm == 0:
                    continue
                angle_rad = (2 * math.pi - (math.radians(angle_deg) % (2 * math.pi))) % (2 * math.pi)
                index = int(angle_rad / self.angle_increment)
                distance_m = distance_mm / 1000.0
                if 0 < index < num_readings:
                    ranges[index] = distance_m

            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'laser_link'
            msg.angle_min = self.angle_min
            msg.angle_max = self.angle_max
            msg.angle_increment = self.angle_increment
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = 0.05
            msg.range_max = self.range_max
            msg.ranges = ranges
            self.get_logger().info("pub info")
            self.publisher_.publish(msg)

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()