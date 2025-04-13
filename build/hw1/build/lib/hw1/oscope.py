import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import math
import time



# The idiom in ROS2 is to encapsulate everything to do with the node in a class that
# is derived from the Node class in rclpy.node.
class oscope(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('oscope')

		self.oscope_pub = self.create_publisher(Float32, 'Sinewave', 10)
		self.slow_wave_pub = self.create_publisher(Float32, 'slow_wave', 10)
		self.fast_wave_pub = self.create_publisher(Float32, 'fast_wave', 10)
		
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
        # recording the start time
		self.start_time = self.get_clock().now().nanoseconds / 1e9

	

	# This callback will be called every time the timer fires.
	def timer_callback(self):
		current_time = self.get_clock().now().nanoseconds / 1e9
		delta_t = current_time - self.start_time
		
		msg_oscope = Float32()
		msg_slow = Float32()
		msg_fast = Float32()
		
		msg_oscope.data = math.sin(2 * math.pi * 1 * delta_t)
		msg_slow.data = math.sin(2 * math.pi * 0.5 * delta_t)
		msg_fast.data = math.sin(2 * math.pi * 2 * delta_t)

		# Publish the message, just like we do in ROS.
		self.oscope_pub.publish(msg_oscope)
		self.slow_wave_pub.publish(msg_slow)
		self.fast_wave_pub.publish(msg_fast)

		# Log that we published something.  In ROS2, loggers are associated with nodes, and
		# the idiom is to use the get_logger() call to get the logger.  This has functions
		# for each of the logging levels.
		self.get_logger().debug('Published: oscope=%f, slow_wave=%f, fast_wave=%f' %
                                  (msg_oscope.data, msg_slow.data, msg_fast.data))


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	publisher = oscope()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(publisher)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()