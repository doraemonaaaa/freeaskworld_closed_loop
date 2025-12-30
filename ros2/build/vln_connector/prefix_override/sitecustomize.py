import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pengyh/workspace/FreeAskAgent/closed_loop/ros2/install/vln_connector'
