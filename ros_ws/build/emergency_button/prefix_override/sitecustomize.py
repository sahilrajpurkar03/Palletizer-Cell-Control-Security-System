import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sahil-dev/Sereact/bin-picking-cell-control/ros_ws/install/emergency_button'
