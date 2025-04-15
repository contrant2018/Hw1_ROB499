#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

def limit_value(value: float) -> float:
    """
    Limit a value to the range [-0.5, 0.5].
    """
    return max(min(value, 0.5), -0.5)

class Limiter(Node):
    def __init__(self, node_name: str, input_topic: str, output_topic: str):
        """
        A generic limiter node.
        :param node_name: Name of the node.
        :param input_topic: Topic name to subscribe for input.
        :param output_topic: Topic name to publish the limited result.
        """
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            Float32, input_topic, self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32, output_topic, 10)
        self.get_logger().info(f"Limiter node started for {input_topic}")

    def listener_callback(self, msg):
        limited = limit_value(msg.data)
        new_msg = Float32()
        new_msg.data = limited
        self.publisher.publish(new_msg)
        self.get_logger().info(f"Received: {msg.data:.3f} from input, limited to {limited:.3f}")

# Main functions for the three limiter nodes:
def main_osc(args=None):
    """
    Limit the regular sine wave from 'Sinewave' and publish to 'limited_wave'.
    """
    rclpy.init(args=args)
    node = Limiter('limiter_osc', 'Sinewave', 'limited_wave')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_slow(args=None):
    """
    Limit the slow sine wave from 'slow_wave' and publish to 'limited_slow_wave'.
    """
    rclpy.init(args=args)
    node = Limiter('limiter_slow', 'slow_wave', 'limited_slow_wave')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_fast(args=None):
    """
    Limit the fast sine wave from 'fast_wave' and publish to 'limited_fast_wave'.
    """
    rclpy.init(args=args)
    node = Limiter('limiter_fast', 'fast_wave', 'limited_fast_wave')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Default to limiting the regular sine wave if run directly.
    main_osc()
