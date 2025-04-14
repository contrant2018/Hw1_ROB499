#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Limiter(Node):
    def __init__(self):
        super().__init__('limiter')

        # subscriber for regular wave
        self.subscription = self.create_subscription(
            Float32, 'Sinewave', self.osc_callback, 10)
        # subscriber for slow wave
        self.sub_slow = self.create_subscription(
            Float32, 'slow_wave', self.slow_callback, 10)
        # subscriber for fast wave
        self.sub_fast = self.create_subscription(
            Float32, 'fast_wave', self.fast_callback, 10)
        
        # publishers for all three (slow, fast and regular)
        self.pub_slow = self.create_publisher(Float32, 'limited_slow_wave', 10)
        self.pub_fast = self.create_publisher(Float32, 'limited_fast_wave', 10)
        self.pub_osc= self.create_publisher(Float32, 'limited_wave', 10)
    
    # limiting values
    def limit_value(self, value):
        """Limit a value to the range [-0.5, 0.5]."""
        return max(min(value, 0.5), -0.5)

    #limiting our regular wave
    def osc_callback(self, msg):
        #recived values
        limited = self.limit_value(msg.data)

        #new message with limited values
        new_msg = Float32()
        new_msg.data = limited

        # publishing the limiting values
        self.pub_osc.publish(new_msg)

        #logging said values
        self.get_logger().info(f"OSC: Received {msg.data:.3f}, limited to {limited:.3f}")

    def slow_callback(self, msg):
        limited = self.limit_value(msg.data)
        new_msg = Float32()
        new_msg.data = limited
        self.pub_slow.publish(new_msg)
        self.get_logger().info(f"SLOW: Received {msg.data:.3f}, limited to {limited:.3f}")

    def fast_callback(self, msg):
        limited = self.limit_value(msg.data)
        new_msg = Float32()
        new_msg.data = limited
        self.pub_fast.publish(new_msg)
        self.get_logger().info(f"FAST: Received {msg.data:.3f}, limited to {limited:.3f}")



def main(args=None):
    rclpy.init(args=args)
    #creating a node instance
    node = Limiter()
    #keep it running
    rclpy.spin(node)

    #clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
