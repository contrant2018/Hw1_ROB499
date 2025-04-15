#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

class SineWavePublisher(Node):
    def __init__(self, node_name: str, topic: str, frequency: float):
        """
        A generic sine wave publisher.
        :param node_name: Name of the node.
        :param topic: Topic name to publish on.
        :param frequency: Sine wave frequency in Hz.
        """
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Float32, topic, 10)
        # Timer period to publish at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.frequency = frequency

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        msg = Float32()
        msg.data = math.sin(2 * math.pi * self.frequency * elapsed)
        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published on '{self.publisher_.topic_name}' value: {msg.data:.3f}")

# Main functions for the three publishers:
def main_osc(args=None):
    """
    Publish the regular (1Hz) sine wave on 'Sinewave'.
    """
    rclpy.init(args=args)
    node = SineWavePublisher('oscope_osc', 'Sinewave', 1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_slow(args=None):
    """
    Publish the slow (0.5Hz) sine wave on 'slow_wave'.
    """
    rclpy.init(args=args)
    node = SineWavePublisher('oscope_slow', 'slow_wave', 0.5)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_fast(args=None):
    """
    Publish the fast (2Hz) sine wave on 'fast_wave'.
    """
    rclpy.init(args=args)
    node = SineWavePublisher('oscope_fast', 'fast_wave', 2)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # If run directly, default to normal sine wave publisher.
    main_osc()
