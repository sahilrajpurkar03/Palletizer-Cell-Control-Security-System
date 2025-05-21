#!/bin/bash

# Start all components in background processes
cd ros_ws/
source install/setup.bash

# ROS Nodes
ros2 run barcode_scanner scanner_node &
ros2 run door_handle door_node &
ros2 run emergency_button ebutton_node &
ros2 run stack_light light_node &

# API Servers
cd ../api_server/server
python3 server.py &
cd ../client
python3 client.py &

# HMI
cd ../../web_hmi/backend
python3 main.py &

# Keep script running
wait