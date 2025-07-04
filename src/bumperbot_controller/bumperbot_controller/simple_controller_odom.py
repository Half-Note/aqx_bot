#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from tf_transformations import quaternion_from_euler
from rclpy.time import Time
from rclpy.constants import S_TO_NS

class SimpleControllerOdom(Node):
    def __init__(self):
        super().__init__('simple_controller_odom')

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_separation").value

        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.prev_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = TransformBroadcaster(self)
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        if len(msg.position) < 2:
            self.get_logger().warn("JointState message has less than 2 positions.")
            return

        msg_time = Time.from_msg(msg.header.stamp)
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.get_logger().warn("Received joint_states with zero timestamp, using current time instead.")
            msg_time = self.get_clock().now()

        dt = msg_time - self.prev_time
        dt_sec = dt.nanoseconds / S_TO_NS
        self.get_logger().debug(f"msg_time: {msg_time.nanoseconds}, prev_time: {self.prev_time.nanoseconds}, dt_sec: {dt_sec}")

        if dt_sec <= 0:
            self.get_logger().warn(f"Non-positive dt_sec: {dt_sec}, skipping update.")
            return

        self.prev_time = msg_time

        right_pos = msg.position[0]
        left_pos = msg.position[1]
        self.get_logger().debug(f"Positions - Right: {right_pos}, Left: {left_pos}")

        d_right = right_pos - self.prev_right_pos
        d_left = left_pos - self.prev_left_pos
        self.get_logger().debug(f"Delta positions - d_right: {d_right}, d_left: {d_left}")

        self.prev_right_pos = right_pos
        self.prev_left_pos = left_pos

        v_right = self.wheel_radius * (d_right / dt_sec)
        v_left = self.wheel_radius * (d_left / dt_sec)

        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        self.get_logger().debug(f"Velocities - v_right: {v_right}, v_left: {v_left}, v: {v}, omega: {omega}")

        ds = (self.wheel_radius * (d_right + d_left)) / 2.0
        dtheta = (self.wheel_radius * (d_right - d_left)) / self.wheel_base

        self.theta += dtheta
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        self.get_logger().debug(f"Pose update - x: {self.x}, y: {self.y}, theta: {self.theta}")

        now_msg = msg_time.to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published odom at time {now_msg.sec}.{now_msg.nanosec}")

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        self.get_logger().info(f"Published TF from odom to base_footprint at time {now_msg.sec}.{now_msg.nanosec}")

def main():
    rclpy.init()
    node = SimpleControllerOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
