import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import time
import random

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')

        # Publishers
        self.robot_position_pub = self.create_publisher(Point, '/robot_position', 10)
        self.actor_position_pub = self.create_publisher(Point, '/actor_position', 10)
        self.robot_goal_pub = self.create_publisher(Point, '/robot_goal', 10)
        self.goal_status_pub = self.create_publisher(Int32, '/robot_goal_status', 10)

        # Initialise Coordinates
        self.robot_position = Point()
        self.actor_position = Point()
        self.robot_goal = Point()
        self.goal_start_time = None

        # Each start position should be unique
        self.generate_initial_point()

        # Timer to publish positions every second
        self.timer = self.create_timer(1.0, self.update_positions)
        
    def generate_initial_point(self):
    #Randomly make unique start positions
        existing_points = set()

        def generate_unique_points():
            while True:
                x = round(random.uniform(0, 10), 1)
                y = round(random.uniform(0, 10), 1)
                if (x, y) not in existing_points:
                    existing_points.add((x, y))
                    return Point(x=x, y=y, z=0.0)

        # Assign initial positions
        self.robot_position = generate_unique_points()
        self.actor_position = generate_unique_points()
        self.robot_goal = generate_unique_points()
        
        #Once assigned I will start the timer
        self.goal_start_time = time.time() 
        #might change so that it doesn't start instantly I would rather the map load in and maybe a start button is pressed to begin the simulation talk to iran

    def update_positions(self):
        # Actor has random movement, this is where I simulate it
        self.actor_position.x = round(random.uniform(0, 10), 1)
        self.actor_position.y = round(random.uniform(0, 10), 1)

        # Publish all points
        self.robot_position_pub.publish(self.robot_position)
        self.actor_position_pub.publish(self.actor_position)
        self.robot_goal_pub.publish(self.robot_goal)

        # Log positions
        self.get_logger().info(f"Robot Position: {self.robot_position}")
        self.get_logger().info(f"Actor Position: {self.actor_position}")
        self.get_logger().info(f"Robot Goal Position: {self.robot_goal}")

def main(args=None):
    rclpy.init(args=args)
    node = PositionPublisher()
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
