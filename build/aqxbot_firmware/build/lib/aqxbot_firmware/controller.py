import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

class UdpCmdVelBridge(Node):
    def __init__(self):
        super().__init__('udp_cmd_vel_bridge')

        # Declare and read parameters
        self.declare_parameter('robot_ip', '192.168.1.100')  # IP of the robot
        self.declare_parameter('robot_port', 8201)           # Port on the robot

        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value

        # Set up UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(f"UDP bridge ready. Sending to {self.robot_ip}:{self.robot_port}")

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular components
        linear = msg.linear.x
        angular = msg.angular.z

        # Convert to string format (customize as needed)
        message = f"{linear:.2f},{angular:.2f}"
        self.get_logger().info(f"Sending: {message}")

        # Send via UDP
        try:
            self.sock.sendto(message.encode(), (self.robot_ip, self.robot_port))
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UdpCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
