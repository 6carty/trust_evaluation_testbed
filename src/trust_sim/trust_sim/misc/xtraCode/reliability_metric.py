import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Point

class ReliabilityMetric(Node):
    def __init__(self):
        super().__init__('reliability_metric')

        # Subscribers
        self.create_subscription(Int32, '/robot_goal_status', self.goal_status_callback, 10) # might need to change the name of this
        self.create_subscription(Bool, '/collision_status', self.collision_callback, 10)
        self.create_subscription(Point, '/robot_position', self.robot_position_callback, 10)
        self.create_subscription(Point, '/robot_goal_position', self.robot_goal_callback, 10)

        # Publisher for reliability metric
        self.reliability_pub = self.create_publisher(Float32, '/metrics/reliability', 10)

        # Variables to track metrics
        self.total_goals = 0
        self.successful_goals = 0
        self.failed_goals = 0
        self.current_collision = False
        self.time_limit = 10.0 #Update dynamically later using services

    def goal_status_callback(self, msg):
        self.total_goals += 1
        if msd.data ==1: #The goal is in progress
            elapsed_time = time.time() - self.goal_start_time
            if elapsed_time>self.time_limit:
                self.get_logger().info("Failed to reach goal in the time limit")
                self.failed_goals +=1
                self.publish_goal_status(0)
                return
            if self.current_collision:
                self.get_logger().info("Failed to reach goal due to a collision")
                self.failed_goals +=1
                self.publish_goal_status(0)
                return
        
        elif msg.data == 2:  #The goal was reached
            self.successful_goals += 1
            self.get_logger().info("Goal was reached successfully.")
            
        # Calculate and publish reliability
        self.calculate_reliability()
        
        
    def publish_goal_status(self, status):
        self.goal_status_pub.publish(Int32(status))
        status_info = {0: "Failure", 1: "In Progress", 2: "Success"}.get(status, "Unknown")
        self.get_logger().info(f"Published goal status: {status_info}")
    
    def collision_callback(self, msg):
        self.current_collision = msg.data
        if self.current_collision:
            self.get_logger().info("Collision detected!")

    def robot_position_callback(self, msg):
        #Potentially use to get the position of the robot to allow the robot to be within a certain range and then publish a success to the /robot_goal_status topic code 2
        pass

    def robot_goal_callback(self, msg):
        #would work with the above idea as we can use it to evaluate how far the robot is from the goal
        pass

    def calculate_reliability(self):
        #Calculating the reliability metric
        if self.total_goals == 0:
            reliability = 0.0
        else:
            reliability = (self.successful_goals / self.total_goals) * 100.0

        # Publishing the reliability metric
        self.reliability_pub.publish(Float32(data=reliability))
        self.get_logger().info(f"Reliability: {reliability:.2f}% ({self.successful_goals}/{self.total_goals})")

def main(args=None):
    rclpy.init(args=args)
    node = ReliabilityMetric()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Shutting down node.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
