import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Float32

class ReliabilityService(Node):

    def __init__(self):
        super().__init__('reliability_service')

        # Service to compute reliability on request
        self.srv = self.create_service(Trigger, '/compute_reliability', self.compute_reliability_callback)

        # Subscriber to track goal status
        self.create_subscription(Int32, '/robot_goal_status', self.goal_status_callback, 10)

        # Publisher for reliability metric
        self.reliability_pub = self.create_publisher(Float32, '/metrics/reliability', 10)

        # Reliability tracking variables
        self.total_goals = 0
        self.successful_goals = 0
        self.failed_goals = 0

        self.get_logger().info("Reliability service is running.")

    def goal_status_callback(self, msg):
        
        self.total_goals += 1
        
        if msg.data == 2:  # Goal reached
            self.successful_goals += 1
            self.get_logger().info("Goal successfully completed.")

        elif msg.data == 0:  # Goal failed
            self.failed_goals += 1
            self.get_logger().info("Goal failed.")

        #Should I compute reliability after each update?
        self.compute_reliability()

    def compute_reliability(self):

        if self.total_goals == 0:
            reliability = 100.0

        else:
            reliability = 100.0 - ((self.failed_goals / self.total_goals) * 100.0)

        # Publish reliability as a topic
        self.reliability_pub.publish(Float32(data=reliability))
        self.get_logger().info(f"Reliability: {reliability:.2f}%     Total Goals: {self.total_goals})")

    def compute_reliability_callback(self, request, response):
        self.compute_reliability()
        response.success = True
        response.message = f"Reliability: {self.successful_goals}/{self.total_goals} ({self.successful_goals/self.total_goals*100 if self.total_goals>0 else 0:.2f}%)"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityService()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Reliability Service shutting down...")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
