import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import socket
import json
import math

class MPUReceiver(Node):
    def __init__(self):
        super().__init__('mpu_udp_receiver')

        # Set up UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 8102))
        self.sock.settimeout(0.1)

        # ROS publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Timer to poll socket
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            imu_data = json.loads(data)

            accel = imu_data.get("accel", [0.0, 0.0, 0.0])
            gyro = imu_data.get("gyro", [0.0, 0.0, 0.0])
            mag = imu_data.get("mag", [0.0, 0.0, 0.0])
            temp = imu_data.get("temp", 0.0)

            # === IMU message ===
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]

            # No orientation (unset)
            imu_msg.orientation_covariance[0] = -1.0

            self.imu_pub.publish(imu_msg)

            # === Magnetic field message ===
            mag_msg = MagneticField()
            mag_msg.header.stamp = imu_msg.header.stamp
            mag_msg.header.frame_id = 'imu_link'
            mag_msg.magnetic_field.x = mag[0]
            mag_msg.magnetic_field.y = mag[1]
            mag_msg.magnetic_field.z = mag[2]

            self.mag_pub.publish(mag_msg)

            self.get_logger().info(f"IMU published | Temp: {temp:.2f}°C")

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MPUReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
