import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import Bool

# from sklearn.cluster import KMeans

class WaypointGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_generator')
        self.map = None

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.wp_pub = self.create_publisher(PoseStamped, '/next_waypoint', 10)

        self.init_timer = self.create_timer(3.0, self.first_tick)
        self.create_subscription(Bool, "/waypoint_reached", self.reached_callback, 10)


        # memory for previously visited waypoints
        self.visited_points = []

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info("updated Map Received!")

    def first_tick(self):
        if self.map is None:
            return
        self.tick()                 # generate first waypoint
        self.init_timer.cancel()    # stop timer forever

    def is_far_enough(self, cx, cy):
        min_dist = 2.0  # meters
        for vx, vy in self.visited_points:
            dist = np.hypot(cx - vx, cy - vy)
            world_dist = dist * self.map.info.resolution

            if world_dist < min_dist:
                return False
        return True
    
    def reached_callback(self, msg):
        if msg.data:       # only trigger on True
            self.tick()

    def tick(self):
        if self.map is None:
            return
        
        grid = np.array(self.map.data).reshape((self.map.info.height,
                                                self.map.info.width))

        # Find unexplored cells (-1)
        unexplored = np.argwhere(grid == -1)
        if unexplored.size == 0:
            self.get_logger().info("No unexplored cells left.")
            return
        
        # Downsample: cluster using kmeans
        # kmeans = KMeans(n_clusters=5, n_init='auto').fit(unexplored)
        # centroids = kmeans.cluster_centers_

        # Cluster via coarse grid (10x10 block)
        block_size = 50
        h, w = grid.shape

        centroids = []

        for i in range(0, h, block_size):
            for j in range(0, w, block_size):
                block = grid[i:i+block_size, j:j+block_size]
                if np.any(block == -1):  # contains unexplored cells
                    cx = i + block.shape[0] // 2
                    cy = j + block.shape[1] // 2
                    centroids.append((cx, cy))

        if len(centroids) == 0:
            return

        # Filter out centroids that are too close to previously visited points
        filtered = [c for c in centroids if self.is_far_enough(c[0], c[1])]

        if len(filtered) == 0:
            self.get_logger().info(f"No sufficiently far centroids found. Skipping. {centroids}")
            return

        # Choose farthest-from-visited among filtered
        if len(self.visited_points) == 0:
            cx, cy = filtered[0]
        else:
            cx, cy = max(
                filtered,
                key=lambda p: min(np.hypot(p[0] - v[0], p[1] - v[1]) for v in self.visited_points)
            )

        # store for future avoidance
        self.visited_points.append((cx, cy))
        self.get_logger().info(f"Visited Points: {self.visited_points}")
        # Pick the nearest centroid to robot OR in order [while using kmean]
        # cx, cy = centroids[0]

        # Convert grid → world coordinates
        row = cx
        col = cy

        wx = col * self.map.info.resolution + self.map.info.origin.position.x
        wy = row * self.map.info.resolution + self.map.info.origin.position.y

        # Create waypoint message
        wp = PoseStamped()
        wp.header.frame_id = "map"
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.pose.position.x = float(wx)
        wp.pose.position.y = float(wy)
        wp.pose.orientation.w = 1.0

        self.wp_pub.publish(wp)
        self.get_logger().info(f"Published waypoint: {wx:.2f}, {wy:.2f}")

def main(args=None):
    rclpy.init(args=args)

    node = WaypointGenerator()
    rclpy.spin(node)

    rclpy.shutdown()

