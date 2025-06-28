import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def main():
    rclpy.init()
    node = Node('static_tf_broadcaster')
    broadcaster = StaticTransformBroadcaster(node)

    static_transform = TransformStamped()
    static_transform.header.stamp = node.get_clock().now().to_msg()
    static_transform.header.frame_id = 'base_link'  # your robot base frame
    static_transform.child_frame_id = 'laser_link'       # frame used in LaserScan

    # Adjust translation/rotation as needed:
    static_transform.transform.translation.x = 0.0
    static_transform.transform.translation.y = 0.0
    static_transform.transform.translation.z = 0.0
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transform)
    node.get_logger().info('Publishing static transform from base_link to laser')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
