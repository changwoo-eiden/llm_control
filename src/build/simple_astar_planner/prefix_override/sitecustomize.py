import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/changwoo/ros2_ws/src/install/simple_astar_planner'
