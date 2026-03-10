#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import random

class GoalSettingService(Node):
    
    def __init__(self):
        super().__init__('goal_setting_service')
        
        # Publisher for sending goals
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Service to assign new goals
        self.goal_service = self.create_service(Trigger, 'set_new_goal', self.handle_goal_request)
        
        # Predefined goal locations (I need to randomise and put them in the bounds of the map)
        self.goal_count = int(input("Enter the number of goals to generate: "))
        self.total_goals_attempted = 0
        self.goal_positions = self.generate_goals(self.goal_count)
        
        self.get_logger().info("Goal Setting Service Ready")
        
        #input("Press Enter to begin...")
        #self.handle_goal_request(self,None,None)
        
    def generate_goals(self, num):
        goals= set()
        min_x, max_x = -1.5, 2.0
        min_y, max_y = -1.0, 1.5
        while len(goals) < num:
            x= round(random.uniform(min_x, max_x),2)
            y= round(random.uniform(min_y, max_y),2)
            if (x,y) not in goals:
                goals.add((x, y))
        return list(goals)

    def handle_goal_request(self, request, response):
        
        if self.total_goals_attempted == self.goal_count :
            self.get_logger().info("No More Goals.")
            response.success = False
            response.message = "All goals have been attempted."
            return response

        goal_x, goal_y = self.goal_positions.pop(0)

 

        # Publish the new goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0 

        self.goal_publisher.publish(goal_msg)

        # Log the attempts to check if working
        self.total_goals_attempted += 1
        self.get_logger().info(f"New Goal Set: ({goal_x}, {goal_y})")
        
        response.success = True
        response.message = f"Goal set to ({goal_x}, {goal_y})."
        return response

def main(args=None):

    rclpy.init(args=args)
    node = GoalSettingService()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Goal Setting Service shutting down...")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
