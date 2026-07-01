import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/erick/ros_ws/src/ROS2_Tello/ros2_tello/install/ros2_tello'
