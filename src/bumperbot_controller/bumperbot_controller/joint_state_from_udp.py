#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import math

class JointStateFromUDP(Node):
    def __init__(self):
        super().__init__('joint_state_from_udp')

        # Encoder setup
        self.TICKS_PER_REV = 20
        self.WHEEL_RADIUS = 0.033
        self.joint_names = ['right_wheel_joint', 'left_wheel_joint']
        self.prev_ticks = [None, None]  # right, left

        # UDP setup
        self.UDP_IP = '0.0.0.0'
        self.UDP_PORT = 8104  # Must match sender
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.setblocking(False)

        # ROS publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.read_udp)

    def ticks_to_radians(self, ticks):
        return 2 * math.pi * (ticks / self.TICKS_PER_REV)

    def read_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            msg = data.decode().strip()  # Example: L:1234,R:1250
            parts = msg.split(',')
            left_ticks = int(parts[0].split(':')[1])
            right_ticks = int(parts[1].split(':')[1])

            left_rad = self.ticks_to_radians(left_ticks)
            right_rad = self.ticks_to_radians(right_ticks)

            joint_msg = JointState()
            joint_msg.name = self.joint_names
            joint_msg.position = [right_rad, left_rad]
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(joint_msg)

        except BlockingIOError:
            pass  # No new UDP message yet

def main():
    rclpy.init()
    node = JointStateFromUDP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
