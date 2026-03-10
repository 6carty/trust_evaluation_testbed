import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')

        # Subscribers
        self.robot_position_sub = self.create_subscription(
            Point, '/robot_position', self.robot_position_callback, 10)
        self.actor_position_sub = self.create_subscription(
            Point, '/actor_position', self.actor_position_callback, 10)
        self.robot_goal_sub = self.create_subscription(
            Point, '/robot_goal', self.robot_goal_callback, 10)

        # Just to allocate memory for them
        self.robot_position = None
        self.actor_position = None
        self.robot_goal = None

    def robot_position_callback(self, msg):
        #Obtaining robots current position
        self.robot_position = msg
        self.get_logger().info(f"Received Robot Position: {self.robot_position}")

    def actor_position_callback(self, msg):
        #Obtaining actors current position
        self.actor_position = msg
        self.get_logger().info(f"Received Actor Position: {self.actor_position}")

    def robot_goal_callback(self, msg):
        #Obtain position of the robots goal
        self.robot_goal = msg
        self.get_logger().info(f"Received Robot Goal Position: {self.robot_goal}")


def main(args=None):
    rclpy.init(args=args)
    node = PositionSubscriber()
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
