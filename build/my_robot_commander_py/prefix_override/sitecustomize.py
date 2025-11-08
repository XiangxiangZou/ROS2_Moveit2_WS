import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zouxx/MyData/ROS2_Moveit2_WS/install/my_robot_commander_py'
