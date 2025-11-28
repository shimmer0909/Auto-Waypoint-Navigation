import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')
        self.create_subscription(PoseStamped, '/next_waypoint', self.wp_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigatte_to_pose')

    def wp_callback(self, wp):
        self.get_logger().info(f"Executing waypoint: {wp.pose.position.x:.2f}, {wp.pose.position.y:.2f}")
        goal = NavigateToPose.Goal()
        goal.pose = wp

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_done)

    def goal_done(self, future):
        result = future.result().get_result()
        self.get_logger().info("Reached waypoint.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutor()
    rclpy.spin(node)