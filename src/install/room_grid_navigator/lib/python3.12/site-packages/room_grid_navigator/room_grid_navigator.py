import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
import math


class RoomGridNavigator(Node):
    def __init__(self):
        super().__init__('room_grid_navigator')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)

        # Room bounds
        self.bottom_left = (-9.11, -5.38)
        self.top_right = (-2.99, -2.0)

        # TF listener for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to RViz goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f"Received goal from RViz: ({x:.2f}, {y:.2f})")

        if self.is_inside_room(x, y):
            self.get_logger().info("Goal is inside the room. Proceeding with entry and grid exploration.")
            self.go_to_entry_point_and_explore(x, y)
        else:
            self.get_logger().info("Goal is outside the room. Ignoring.")

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
        self.get_logger().info(f"Published {len(waypoint_list)} grid point markers")

    def go_to_entry_point_and_explore(self, entry_x, entry_y):
        self.nav_to_pose_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.to_pose(entry_x, entry_y)
        self.get_logger().info(f"Navigating to entry point: ({entry_x:.2f}, {entry_y:.2f})")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_nav_to_pose_complete(f))

    def _on_nav_to_pose_complete(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Initial navigation goal rejected')
            return
        self.get_logger().info('Initial goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_reach_room_and_explore())

    def _on_reach_room_and_explore(self):
        self.get_logger().info('Reached entry point. Starting nearest-neighbor grid navigation.')
        grid_points = self.generate_grid(3, 3)
        self.publish_markers(grid_points)
        self.visited = []
        self.unvisited = [self.to_pose(x, y) for x, y in grid_points]
        self.send_next_nearest()

    def get_current_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                                    timeout=Duration(seconds=2.0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot position: {e}")
            # fallback: use origin or last known point
            return 0.0, 0.0

    def send_next_nearest(self):
        if not self.unvisited:
            self.get_logger().info("ompleted all waypoints.")
            return

        robot_x, robot_y = self.get_current_robot_position()

        # Find nearest unvisited waypoint
        def distance(pose):
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            return math.hypot(dx, dy)

        next_pose = min(self.unvisited, key=distance)
        self.unvisited.remove(next_pose)
        self.visited.append(next_pose)

        self.get_logger().info(f"Navigating to waypoint at ({next_pose.pose.position.x:.2f}, "
                               f"{next_pose.pose.position.y:.2f})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = next_pose

        self.nav_to_pose_client.wait_for_server()
        self._nav_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._nav_future.add_done_callback(self.goal_response_callback_nearest)

    def goal_response_callback_nearest(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            return
        self.get_logger().info("Navigation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback_nearest)

    def result_callback_nearest(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Reached waypoint. Performing action...")
            self.perform_action_at_waypoint()
        else:
            self.get_logger().error(f"Failed to reach waypoint. Code: {result.error_code}, Msg: '{result.error_msg}'")
            self.send_next_nearest()

    def perform_action_at_waypoint(self):
        self.get_logger().info("Simulating sensor reading or action...")

        # Replace this with actual logic (e.g., air quality reading, data logging, camera snapshot)
        self.action_timer = self.create_timer(2.0, self.action_complete_callback)

    def action_complete_callback(self):
        self.get_logger().info("Action complete. Proceeding to next waypoint.")
        self.destroy_timer(self.action_timer)
        self.send_next_nearest()


def main():
    rclpy.init()
    navigator = RoomGridNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
