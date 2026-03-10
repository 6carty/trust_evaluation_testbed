#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from std_srvs.srv import Trigger

from std_msgs.msg import Float32

 

class ComfortService(Node):

    def __init__(self):

        super().__init__('comfort_service')

 

        # Subscribe to LIDAR scans

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

 

        # Service to compute comfort

        self.srv = self.create_service(Trigger, '/compute_comfort', self.compute_comfort_callback)

 

        # Comfort tracking

        self.comfort_score = 100.0  # Start with max comfort

 

        self.get_logger().info("Comfort Service Running...")

 

    def scan_callback(self, msg):

        """Triggered when LIDAR scan is received."""

        min_distance = min(msg.ranges)  # Get closest detected object

 

        if min_distance < 1.5:  # If robot is near something

            if min_distance < 1.0:

                self.comfort_score -= 10  # Major discomfort

            else:

                self.comfort_score -= 5  # Minor discomfort

 

            if self.comfort_score < 0:

                self.comfort_score = 0  # Prevent negative comfort score

 

            self.get_logger().warn(f"Comfort Score Dropped: {self.comfort_score:.2f} (Closest Object: {min_distance:.2f}m)")

 

    def compute_comfort_callback(self, request, response):

        """Returns the current comfort score."""

        response.success = True

        response.message = f"Comfort Score: {self.comfort_score:.2f}"

        self.get_logger().info(response.message)

        return response

 

def main(args=None):

    rclpy.init(args=args)

    node = ComfortService()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info("Shutting down Comfort Service.")

    finally:

        node.destroy_node()

        rclpy.shutdown()

 

if __name__ == '__main__':

    main()

