import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vd/carla-ros-bridge/catkin_ws/install/carla_ad_demo'
