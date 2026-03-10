import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import random
import time

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')

        # Publisher for actor position (simulating human movement)
        self.actor_position_pub = self.create_publisher(Pose, '/actor_position', 10)

        # Publisher for robot goal position
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Publisher to indicate a goal cancellation
        self.goal_cancel_pub = self.create_publisher(Bool, '/goal_cancel', 10)

        # Subscriber to check if the robot reached the goal
        self.create_subscription(Bool, '/goal_status', self.goal_status_callback, 10)

        # Internal variables
        self.actor_position = Pose()
        self.goal_queue = []  # List of pre-generated goal positions
        self.current_goal_index = 0  # Index for tracking current goal
        self.goal_start_time = None  # Track when the current goal started

        # Generate the initial set of goals
        self.generate_goal_positions()

        # Timers to update positions and check goal timeout
        self.create_timer(3.0, self.update_actor_position)  # Update actor every 3 seconds
        self.create_timer(1.0, self.check_goal_timeout)  # Check for goal time limit every second

    def generate_goal_positions(self):
        """Generate a list of random goal positions to be completed sequentially."""
        for _ in range(5):  # Change this number to generate more/less goals
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = round(random.uniform(0, 5), 2)
            goal.pose.position.y = round(random.uniform(0, 5), 2)
            goal.pose.position.z = 0.0
            self.goal_queue.append(goal)

        self.get_logger().info(f"🗺️ Generated {len(self.goal_queue)} goals.")

    def update_actor_position(self):
        """Randomly move the actor to simulate human movement."""
        self.actor_position.position.x = round(random.uniform(0, 5), 2)
        self.actor_position.position.y = round(random.uniform(0, 5), 2)
        self.actor_position.position.z = 0.0

        self.actor_position_pub.publish(self.actor_position)
        self.get_logger().info(f"👤 Actor moved to: {self.actor_position.position}")

    def publish_next_goal(self):
        """Publish the next goal if there are any left in the queue."""
        if self.current_goal_index < len(self.goal_queue):
            self.goal_queue[self.current_goal_index].header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(self.goal_queue[self.current_goal_index])
            self.goal_start_time = time.time()  # Store the start time of the goal
            self.get_logger().info(f"🎯 New Goal Published: {self.goal_queue[self.current_goal_index].pose.position}")
        else:
            self.get_logger().info("✅ All goals have been completed or canceled.")

    def check_goal_timeout(self):
        """Cancel the goal if it exceeds the time limit of 5 seconds."""
        if self.goal_start_time is not None:
            elapsed_time = time.time() - self.goal_start_time
            if elapsed_time > 5.0:  # Time limit exceeded
                self.get_logger().info(f"⏳ Goal {self.current_goal_index + 1} timed out! Cancelling and moving to the next goal.")
                self.goal_cancel_pub.publish(Bool(data=True))  # Publish goal cancellation
                self.current_goal_index += 1  # Move to the next goal
                self.goal_start_time = None  # Reset goal timer
                self.publish_next_goal()  # Send the next goal

    def goal_status_callback(self, msg):
        """Handle goal completion and move to the next goal."""
        if msg.data:  # True means goal reached
            self.get_logger().info(f"🏁 Goal {self.current_goal_index + 1} reached.")
            self.current_goal_index += 1  # Move to the next goal
            self.goal_start_time = None  # Reset the goal timer
            self.publish_next_goal()  # Send the next goal

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

