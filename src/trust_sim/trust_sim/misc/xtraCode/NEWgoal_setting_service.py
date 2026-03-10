import rclpy

from rclpy.node import Node

from std_srvs.srv import Trigger  # Using Trigger instead of SetGoal

from geometry_msgs.msg import Point

import random

 

class GoalSettingService(Node):

    def __init__(self):

        super().__init__('goal_setting_service')

 

        # Service to provide new goals

        self.srv = self.create_service(Trigger, '/set_new_goal', self.handle_goal_request)

 

        # Publisher to send new goal positions

        self.goal_pub = self.create_publisher(Point, '/robot_goal', 10)

 

        # Pre-generate a fixed set of random goals

        self.max_goals = 10

        self.goal_list = self.generate_random_goals(self.max_goals)

        self.goal_index = 0  # Track the current goal

 

        self.get_logger().info("Goal Setting Service Ready.")

 

    def generate_random_goals(self, num_goals):

        """Generate a list of unique goal positions."""

        goals = []

        existing_points = set()

 

        while len(goals) < num_goals:

            x, y = round(random.uniform(0, 10), 1), round(random.uniform(0, 10), 1)

            if (x, y) not in existing_points:

                existing_points.add((x, y))

                goals.append(Point(x=x, y=y, z=0.0))

 

        return goals

 

    def handle_goal_request(self, request, response):

        """Send the next goal from the list when requested."""

        if self.goal_index >= self.max_goals:

            self.get_logger().warn("No more goals available.")

            response.success = False

            response.message = "All goals completed."

            return response

 

        # Get the next goal and publish it

        goal = self.goal_list[self.goal_index]

        self.goal_pub.publish(goal)

        self.get_logger().info(f"New Goal Published: {goal.x}, {goal.y}")

       

        # Move to the next goal

        self.goal_index += 1

 

        response.success = True

        response.message = f"Goal {self.goal_index} set."

        return response

 

def main(args=None):

    rclpy.init(args=args)

    node = GoalSettingService()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info("Shutting down Goal Setting Service.")

    finally:

        node.destroy_node()

        rclpy.shutdown()

 

if __name__ == '__main__':

    main()
