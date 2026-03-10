#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatusArray
from std_srvs.srv import Trigger
import math

class GoalMonitor(Node):

    def __init__(self):
        super().__init__('goal_monitor')

        # Subscribe to the navigation goal status topic
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status_callback, 10)

        # Subscribe to the robot movement
        self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        
        #SUbscribe to robot position 
        self.create_subscription(PoseStamped, '/amcl_pose', self.robot_pos_callback, 10)
        
        # Service client for goal setting
        self.goal_client = self.create_client(Trigger, '/set_new_goal')

        # Publisher for goal status
        self.goal_status_pub = self.create_publisher(Int32, '/robot_goal_status', 10)

        # Wait for service to be available
        while not self.goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /set_new_goal service...")

        self.robot_moving = False
        self.robot_pos = None
        self.robot_goal = None
        
        
        self.get_logger().info("Goal Monitor Initialized.")
        
    def velocity_callback(self, msg):
    # Nav2 falsely reports aborted so this checks to see if the robot still tries to get to the goal or not
        moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        self.robot_moving = moving
   
    def robot_pos_callback(self, msg):
        self.robot_pos = msg.pose.position

    def goal_status_callback(self, msg):
        
        if not msg.status_list:
            return  # No status update yet

        status = msg.status_list[-1].status  # The latest status
 
        if status == 4:  # SUCCESS
            self.get_logger().info("Goal Reached Successfully! Requesting New Goal...")
            self.goal_status_pub.publish(Int32(data=2))  # 2 = success
            self.request_new_goal()
            
        elif status == 5:  # ABORTED
            if self.check_goal_reached():
                self.get_logger().info("False Abort, Goal Success Detected")
                self.goal_status_pub.publish(Int32(data=2))
            elif self.robot_moving():
                self.get_logger().warn("False Abort Detected, Robot Still Moving")
                return
            else:
                self.get_logger().warn("True Abort Detected, Requesting New Goal...")
                self.goal_status_pub.publish(Int32(data=0))  # 0 = failure
                self.request_new_goal()    

        elif status == 6:  # FAILED
            self.get_logger().warn("Goal Failed. Requesting New Goal...")
            self.goal_status_pub.publish(Int32(data=0))  # 0 = failure
            self.request_new_goal()

    def check_goal_reached(self):
        if self.robot_pos and self.robot_goal:
            dist= math.sqrt((self.robot_position.x - self.current_goal.x)**2 + (self.robot_position.y - self.current_goal.y)**2)
            return dist <= 0.2 #Goal Acceptance tolerance?
        return False


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
