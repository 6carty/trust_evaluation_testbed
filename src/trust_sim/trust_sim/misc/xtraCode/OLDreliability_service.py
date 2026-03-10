import rclpy

from rclpy.node import Node

from std_srvs.srv import Trigger

from std_msgs.msg import Int32

 

class ReliabilityService(Node):

    def __init__(self):

        super().__init__('reliability_service')

 

        # Service to compute reliability

        self.srv = self.create_service(Trigger, '/compute_reliability', self.compute_reliability_callback)

 

        # Subscriber to track goal status

        self.create_subscription(Int32, '/robot_goal_status', self.goal_status_callback, 10)

 

        # Reliability tracking variables

        self.total_goals = 0

        self.successful_goals = 0

        self.failed_goals = 0

 

        self.get_logger().info("Reliability service is running.")

 

    def goal_status_callback(self, msg):

        """Track the goal outcomes"""

        self.total_goals += 1

 

        if msg.data == 2:  # Goal successfully reached

            self.successful_goals += 1

            self.get_logger().info("Goal successfully completed.")

 

        elif msg.data == 0:  # Goal failed

            self.failed_goals += 1

            self.get_logger().info("Goal failed.")

 

    def compute_reliability_callback(self, request, response):

        """Compute reliability when the service is called"""

        if self.total_goals == 0:

            reliability = 0.0

        else:

            reliability = (self.successful_goals / self.total_goals) * 100.0

 

        response.success = True

        response.message = f"Reliability: {reliability:.2f}% ({self.successful_goals}/{self.total_goals})"

 

        self.get_logger().info(response.message)

        return response

 

def main(args=None):

    rclpy.init(args=args)

    node = ReliabilityService()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info("Shutting down Reliability Service.")

    finally:

        node.destroy_node()

        rclpy.shutdown()

 

if __name__ == '__main__':

    main()
