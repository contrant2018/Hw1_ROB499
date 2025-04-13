import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/Ubuntu/ros2_ws/src/hw1/install/hw1'
