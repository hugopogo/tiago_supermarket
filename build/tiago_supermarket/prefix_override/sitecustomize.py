import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rosdev/ros2_ws/src/tiago_supermarket/install/tiago_supermarket'
