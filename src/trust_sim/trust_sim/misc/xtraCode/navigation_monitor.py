import rclpy

from rclpy.node import Node

from std_msgs.msg import Int32

from action_msgs.msg import GoalStatusArray

 

class NavigationMonitor(Node):

    def __init__(self):

        super().__init__('navigation_monitor')

 

        # Subscribe to the navigation goal status topic

        self.create_subscription(

            GoalStatusArray,

            '/navigate_to_pose/_action/status',

            self.status_callback,

            10

        )

 

        self.get_logger().info("Navigation Monitor Initialized.")

 

    def status_callback(self, msg):

        """Process navigation status updates."""

        if not msg.status_list:

            return  # No status updates received yet

 

        # Get the latest status

        status = msg.status_list[-1].status

 

        status_dict = {

            0: "UNKNOWN",

            1: "ACCEPTED",

            2: "EXECUTING",

            3: "CANCELING",

            4: "SUCCEEDED",

            5: "CANCELED/FAILED",

            6: "ABORTED"

        }

 

        status_message = status_dict.get(status, "UNKNOWN")

 

        if status == 4:

            self.get_logger().info("Goal Reached Successfully!")

        elif status == 5 or status == 6:

            self.get_logger().warn("Goal Failed or Aborted.")

        else:

            self.get_logger().info(f"Current Goal Status: {status_message}")

 

def main(args=None):

    rclpy.init(args=args)

    node = NavigationMonitor()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info("Shutting down Navigation Monitor.")

    finally:

        node.destroy_node()

        rclpy.shutdown()

 

if __name__ == '__main__':

    main()
