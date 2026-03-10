import rclpy

from rclpy.node import Node

from std_msgs.msg import Int32, Float32

from geometry_msgs.msg import Point

from action_msgs.msg import GoalStatusArray

from std_srvs.srv import Trigger

 

class GoalMonitor(Node):

    def __init__(self):

        super().__init__('goal_monitor')

 

        # Subscribe to the navigation goal status topic

        self.create_subscription(

            GoalStatusArray,

            '/navigate_to_pose/_action/status',

            self.goal_status_callback,

            10

        )

 

        # Create service client for goal setting

        self.goal_client = self.create_client(Trigger, '/set_new_goal')

 

        # Publisher for new goal positions

        self.goal_pub = self.create_publisher(Point, '/robot_goal', 10)
        
        
        # Publisher for reliability metric

        self.reliability_pub = self.create_publisher(Float32, '/metrics/reliability', 10)

 

        # Reliability tracking

        self.successful_goals = 0

        self.failed_goals = 0

        self.total_goals = 0

 

        # Wait for service

        while not self.goal_client.wait_for_service(timeout_sec=1.0):

            self.get_logger().warn("Waiting for /set_new_goal service...")

 

        self.get_logger().info("Goal Monitor Initialized.")

 

    def goal_status_callback(self, msg):

        """Triggered when navigation status updates."""

        if not msg.status_list:

            return  # No status update yet

 

        status = msg.status_list[-1].status  # Get latest status

 

        if status == 4:  # SUCCEEDED

            self.successful_goals += 1

            self.total_goals += 1

            self.get_logger().info("Goal Reached Successfully! Requesting New Goal...")

            self.request_new_goal()

 

        elif status in (5, 6):  # FAILED OR ABORTED

            self.failed_goals += 1

            self.total_goals += 1

            self.get_logger().warn("Goal Failed. Requesting New Goal...")

            self.request_new_goal()

 

        # Calculate reliability after each goal attempt

        self.calculate_reliability()

 

    def request_new_goal(self):

        """Requests a new goal from the goal setting service."""

        request = Trigger.Request()

        future = self.goal_client.call_async(request)

        future.add_done_callback(self.handle_new_goal_response)

 

    def handle_new_goal_response(self, future):

        """Handles the response from the goal setting service."""

        try:

            response = future.result()

            new_goal = Point(x=response.goal_x, y=response.goal_y, z=0.0)

 

            # Publish the new goal

            self.goal_pub.publish(new_goal)

            self.get_logger().info(f"Published New Goal: {new_goal.x}, {new_goal.y}")

 

        except Exception as e:

            self.get_logger().error(f"Service call failed: {e}")

 

    def calculate_reliability(self):

        """Calculate and log reliability."""

        if self.total_goals == 0:

            reliability = 0.0

        else:

            reliability = (self.successful_goals / self.total_goals) * 100.0

 

        self.get_logger().info(f"Reliability: {reliability:.2f}% ({self.successful_goals}/{self.total_goals})")
        self.reliability_pub.publish(Float32(data=reliability))

 

def main(args=None):

    rclpy.init(args=args)

    node = GoalMonitor()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info("Shutting down Goal Monitor.")

    finally:

        node.destroy_node()

        rclpy.shutdown()

 

if __name__ == '__main__':

    main()
