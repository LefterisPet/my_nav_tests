import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leftpet/ros2_nav_test_ws/install/my_nav_tests'
