#!/usr/bin/env python3
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
        
         # Subscriber to track goal status
        self.create_subscription(Int32, '/robot_distance_remaining', self.distance_callback, 10)

        # Publisher for reliability metric
        self.reliability_pub = self.create_publisher(Float32, '/metrics/reliability', 10)

        # Reliability tracking variables
        self.total_goals = 0
        self.successful_goals = 0
        self.failed_goals = 0
        self.distance_remaining = None
        
        #Reliability Calculation Parameters
        self.max_reliability = 100.0
        self.current_reliability = 100.0
        self.prev_distance = None
        self.reliabiity_increase = 5.0
        self.penalty = 15.0
        
        #Timer to constantly track distance remaining
        self.create_timer(1.0, self.update_distance)

        self.get_logger().info("Reliability service is running.")

    def goal_status_callback(self, msg):
        
        self.total_goals += 1
        
        if msg.data == 2:  # Goal reached
            self.successful_goals += 1
            self.get_logger().info("Goal successfully completed.")
            
           
            self.current_reliability = min(self.current_reliability + self.reliabiity_increase, self.max_reliability)

        elif msg.data == 0:  # Goal failed
            self.failed_goals += 1
            self.get_logger().info("Goal failed.")
            
            if self.prev_distance is not None:
            	self.penalty = min(self.prev_distance * 5.0, 20.0) #Maybe uncap this?
            else:
            	self.penalty = 15.0
            	
            self.max_reliability = max(self.max_reliability - 5.0, (self.successful_goals / self.total_goals))
            
            penalty = 5.0 #Penalty for failure
            self.current_reliability = max(self.current_reliability - self.penalty, 0.0)
        
        self.distance_remaining = None
        self.prev_distance = None
        self.publish_reliability()


    def distance_callback(self,msg):
        #Distance remaining from goal_monitor
        self.distance_remaining = msg.data
        if self.distance_remaining > 0.0:
            self.prev_distance = self.distance_remaining
        
    def update_distance(self):
        if self.distance_remaining is not None:
            self.get_logger().info(f"Updated Distance Remaining: {self.distance_remaining:.2f} meters")

    def publish_reliability(self):
        self.reliability_pub.publish(Float32(data=self.current_reliability))
        self.get_logger().info(f"Current Reliability: {self.current_reliability:.2f}%)")

    def compute_reliability_callback(self, request, response):
        response.success = True
        response.message = f"Reliability: {self.successful_goals}/{self.total_goals} ({self.successful_goals/self.total_goals*100 if self.total_goals>0 else 0:.2f}%)"
        self.get_logger().info(response.message)
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
