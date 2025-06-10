#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
import math

class GoalMonitor(Node):

    def __init__(self):
        super().__init__('goal_monitor')

        # Subscribe to the navigation goal status topic
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status_callback, 10)
        
        # Subscribe to the navigation goal feedback topic to get the distance
        self.create_subscription(NavigateToPose.Feedback, '/navigate_to_pose/_action/feedback', self.nav_feedback_callback, 10)

        # Service client for goal setting
        self.goal_client = self.create_client(Trigger, '/set_new_goal')

        # Publisher for goal status
        self.goal_status_pub = self.create_publisher(Int32, '/robot_goal_status', 10)
        
        
        # Publisher for goal status
        self.distance_pub = self.create_publisher(Int32, '/robot_distance_remaining', 10)

        self.distance_remaining = None 
	#distance between the robot and the goal
	
	
        # Wait for service to be available
        while not self.goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /set_new_goal service...")
        
        self.get_logger().info("Goal Monitor Initialized.")
        
    
    def goal_status_callback(self, msg):
        
        if not msg.status_list:
            return  # No status update yet

        status = msg.status_list[-1].status  # The latest status
 
        if status == 4:  # SUCCESS
            self.get_logger().info("Goal Reached Successfully! Requesting New Goal...")
            self.goal_status_pub.publish(Int32(data=2))  # 2 = success
            self.request_new_goal()
            
        elif status == 5:  # ABORTED
                self.get_logger().warn("Abort Detected, Requesting New Goal...")
                self.goal_status_pub.publish(Int32(data=0))  # 0 = failure  #CHANGE TO ONE AND MAKE 1 BE ABORTING I WANT TO SEE IT
                self.request_new_goal()    

        elif status == 6:  # FAILED
            self.get_logger().warn("Goal Failed. Requesting New Goal...")
            self.goal_status_pub.publish(Int32(data=0))  # 0 = failure
            self.request_new_goal()
            
    def nav_feedback_callback(self,msg):
    #Gets distance between robot + the goal
        self.distance_remaining = msg.feedback.distance_remaining
        self.distance_pub.publish(Float32(data=self.distance_remaining))
        self.get_logger().info(f"Distance Remaining: {self.distance_remaining:.2f} meters")
    
    

    def request_new_goal(self):
        #Requests a new goal from the goal setting service
        request = Trigger.Request()
        future = self.goal_client.call_async(request)
        future.add_done_callback(self.handle_new_goal_response)

    def handle_new_goal_response(self, future):
    #Handles response from goal setting service

        try:
            response = future.result()
            self.get_logger().info(f"New Goal Response: {response.message}")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Goal Monitor shutting down...")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
