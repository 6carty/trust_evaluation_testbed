#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CompositeTrustScoreService(Node):

    def __init__(self):
        super().__init__('composite_trust_score_service')

        # Initial values
        self.reliability = 100.0
        self.comfort = 100.0
        self.safety = 100.0

        # Weighting for each metric (make dynamic so users can adjust)
        self.w_reliability = 0.4
        self.w_comfort = 0.3
        self.w_safety = 0.3

        # Metric subscriptions
        self.create_subscription(Float32, '/metrics/reliability', self.reliability_callback, 10)
        self.create_subscription(Float32, '/metrics/comfort', self.comfort_callback, 10)
        self.create_subscription(Float32, '/metrics/safety', self.safety_callback, 10)

        # Composite trust publisher
        self.trust_pub = self.create_publisher(Float32, '/metrics/trust_score', 10)

        # Timer to compute and publish every second
        self.create_timer(1.0, self.publish_trust_score)

        self.get_logger().info("Composite Trust Score Service is running.")

    def reliability_callback(self, msg):
        self.reliability = msg.data

    def comfort_callback(self, msg):
        self.comfort = msg.data

    def safety_callback(self, msg):
        self.safety = msg.data

    def publish_trust_score(self):
        total_weight = self.w_reliability + self.w_comfort + self.w_safety
        trust_score = (
            self.w_reliability * self.reliability +
            self.w_comfort * self.comfort +
            self.w_safety * self.safety
        ) / total_weight

        self.trust_pub.publish(Float32(data=trust_score))
        self.get_logger().info(f"Composite Trust Score: {trust_score:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CompositeTrustScoreService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Composite Trust Score Service.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
