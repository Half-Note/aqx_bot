import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import math
import time


class RoomGridNavigator(Node):
    def __init__(self):
        super().__init__('room_grid_navigator')

        # Action client to send single navigation goals
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Action server to receive FollowWaypoints goals (triggered by Nav2 RViz panel button)
        self.waypoint_action_server = ActionServer(
            self,
            FollowWaypoints,
            '/follow_waypoints',
            self.execute_waypoint_callback,
            goal_callback=self.goal_response_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publisher to visualize waypoints as markers in RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)

        # Room bounds (example)
        self.bottom_left = (-9.11, -5.38)
        self.top_right = (-2.99, -2.0)

        # TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def goal_response_callback(self, goal_request):
        self.get_logger().info('Received FollowWaypoints goal request.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for FollowWaypoints goal.')
        return CancelResponse.ACCEPT

    def execute_waypoint_callback(self, goal_handle):
        self.get_logger().info('Executing FollowWaypoints action.')

        # Step 1: Get current robot position
        robot_x, robot_y = self.get_current_robot_position()

        if not self.is_inside_room(robot_x, robot_y):
            self.get_logger().warn(f"Robot at ({robot_x:.2f}, {robot_y:.2f}) is not inside the known room bounds.")
            goal_handle.abort()
            return FollowWaypoints.Result()

        self.get_logger().info(f"Robot at ({robot_x:.2f}, {robot_y:.2f}) inside room. Generating waypoints.")

        # Step 2: Generate grid points and publish markers
        grid_points = self.generate_grid(3, 3)
        self.publish_markers(grid_points)

        # Step 3: Navigate to each waypoint sequentially
        for gx, gy in grid_points:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Waypoint following canceled by client.')
                goal_handle.canceled()
                return FollowWaypoints.Result()

            pose = self.to_pose(gx, gy)
            goal = NavigateToPose.Goal()
            goal.pose = pose

            self.get_logger().info(f'Navigating to waypoint ({gx:.2f}, {gy:.2f})')

            self.nav_to_pose_client.wait_for_server()

            send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle_nav = send_goal_future.result()

            if not goal_handle_nav.accepted:
                self.get_logger().error('Navigation goal rejected by Nav2.')
                goal_handle.abort()
                return FollowWaypoints.Result()

            get_result_future = goal_handle_nav.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result().result

            if result.error_code != 0:
                self.get_logger().warn(f'Failed to reach waypoint with error code {result.error_code}. Continuing to next waypoint.')
                continue

            self.get_logger().info('Reached waypoint. Simulating sensor/action wait...')
            self.perform_fake_action_at_waypoint()

        self.get_logger().info('Finished all waypoints.')
        goal_handle.succeed()
        return FollowWaypoints.Result()

    def perform_fake_action_at_waypoint(self):
        # Simulate sensor reading or data collection
        time.sleep(2)  # 2 seconds wait

    def is_inside_room(self, x, y):
        return self.bottom_left[0] <= x <= self.top_right[0] and self.bottom_left[1] <= y <= self.top_right[1]

    def generate_grid(self, rows=3, cols=3):
        min_x, min_y = self.bottom_left
        max_x, max_y = self.top_right
        width = max_x - min_x
        height = max_y - min_y
        cell_w = width / cols
        cell_h = height / rows
        waypoints = []
        for i in range(cols):
            for j in range(rows):
                gx = min_x + (i + 0.5) * cell_w
                gy = min_y + (j + 0.5) * cell_h
                waypoints.append((gx, gy))
        return waypoints

    def to_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def publish_markers(self, waypoint_list):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(waypoint_list):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'grid_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(waypoint_list)} grid point markers.')

    def get_current_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                                    timeout=Duration(seconds=2.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f'Failed to get robot position: {e}')
            # fallback default position
            return 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    navigator = RoomGridNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
