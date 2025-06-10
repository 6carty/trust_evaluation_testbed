#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class SafetyService(Node):

    def __init__(self):
        super().__init__('safety_service')


        # Subscribe to LIDAR scans
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Service to compute safety
        self.srv = self.create_service(Trigger, '/compute_safety', self.compute_safety_callback)

        # Safety tracking
        self.safety_score = 100.0  # Start with max safety

        self.get_logger().info("Safety Service Running...")

 

    def scan_callback(self, msg):
        """Triggered when LIDAR scan is received."""
        min_distance = min(msg.ranges)  # Get closest detected object

        if min_distance < 0.2:  # If robot is near something
            if min_distance < 0.1:
                self.safety_score -= 10  # Major collision
                
            else:
                self.safety_score -= 5  # Near Miss

            if self.safety_score < 0:
                self.safety_score = 0  # Prevent negative safety score

            self.get_logger().warn(f"Safety Score Dropped: {self.safety_score:.2f} (Closest Object: {min_distance:.2f}m)")
 
    def compute_safety_callback(self, request, response):
        response.success = True
        response.message = f"Safety Score: {self.safety_score:.2f}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SafetyService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Safety Service.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
