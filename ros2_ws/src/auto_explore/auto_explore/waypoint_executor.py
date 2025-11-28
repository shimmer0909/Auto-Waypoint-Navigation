import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.create_subscription(PoseStamped, '/next_waypoint', self.wp_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_queue = []
        self.current_goal_active = False

    def wp_callback(self, wp):
        # self.get_logger().info(f"Executing waypoint: {wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}")
        # goal = NavigateToPose.Goal()
        # goal.pose = wp

        # self.nav_client.wait_for_server()
        # future = self.nav_client.send_goal_async(goal)
        # future.add_done_callback(self.goal_done)

        # Add waypoint to queue
        self.get_logger().info(f"Waypoint Recieved: {wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}")
        self.goal_queue.append(wp)
        self.get_logger().info(f"Waypoint Added: {self.goal_queue}")
        # Start processing if not already active
        if not self.current_goal_active:
            self.send_next_goal()

    def send_next_goal(self):
        if len(self.goal_queue) == 0:
            self.current_goal_active = False
            return

        wp = self.goal_queue.pop(0)
        self.get_logger().info(f"Setting next waypoint: {wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}")
        goal = NavigateToPose.Goal()
        goal.pose = wp

        self.nav_client.wait_for_server()
        self.get_logger().info(f"Sending next waypoint: {wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}")

        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.current_goal_active = False
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Reached waypoint.")

        self.current_goal_active = False
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)

    node = WaypointExecutor()
    rclpy.spin(node)

    rclpy.shutdown()
