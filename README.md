# Bin Picking Cell Control System

![System Architecture](docs/system_architecture.png)

A complete implementation of a robotic bin picking cell with ROS 2 control, API server/client, and web-based HMI.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [System Components](#system-components)
- [API Usage](#api-usage)
- [Debugging](#debugging)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Prerequisites

### System Requirements
- **Ubuntu 22.04** (recommended)
- **ROS 2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **Python 3.10+**

### Dependencies
```bash
sudo apt update && sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    build-essential
```

## Installation

### Clone the repository:
```bash
git clone https://github.com/your-repo/bin_picking_cell_control.git
cd bin_picking_cell_control
```

### Install Python dependencies:
```bash
pip install -r requirements.txt
```

### Build ROS workspace:
```bash
cd ros_ws/
colcon build
source install/setup.bash
cd ..
```

## Quick Start

### Launch the Entire System
```bash
chmod +x launch.sh  # Only needed once
./launch.sh
```

This will start:
- All ROS 2 nodes
- API server (port 8080)
- API client (port 8081)
- HMI dashboard (port 8000)

### Access the HMI

Open in your browser:  
[http://localhost:8000](http://localhost:8000)

## System Components

| Component        | Port/Topic           | Description                                  |
|------------------|----------------------|----------------------------------------------|
| WMS API Server   | http://localhost:8080 | Receives pick requests from WMS              |
| Cell API Client  | http://localhost:8081 | Processes pick operations                    |
| HMI Dashboard    | http://localhost:8000 | Real-time monitoring interface               |
| Barcode Scanner  | `/barcode`           | Publishes random 5-digit barcodes            |
| Door Handle      | `/door_status`       | Tracks door open/closed state                |
| Emergency Button | `/e_button_status`   | Monitors emergency stop state                |
| Stack Light      | `/stack_light_status`| Visualizes system status (0: normal, 1: paused, -1: emergency) |

## API Usage

### Send Pick Request
```bash
curl -X POST "http://localhost:8080/pick" \
  -H "Content-Type: application/json" \
  -d '{"pickId": 123, "quantity": 4}'
```

### Expected Successful Response
```json
{
  "pickId": 123,
  "pickSuccessful": true,
  "errorMessage": null,
  "itemBarcode": "58392"
}
```

### Error Responses
```json
{
  "pickId": 123,
  "pickSuccessful": false,
  "errorMessage": "CELL DOOR OPEN",
  "itemBarcode": null
}
```

## Debugging

### Individual Component Checks
```bash
# In separate terminals:
cd ros_ws/
source install/setup.bash

# Check barcode scanner
ros2 run barcode_scanner scanner_node

# Check door status
ros2 run door_handle door_node

# Check emergency button
ros2 run emergency_button ebutton_node

# Check stack light
ros2 run stack_light light_node
```

### ROS 2 Topic Monitoring
```bash
# List all active topics
ros2 topic list

# View specific topic data
ros2 topic echo /door_status
ros2 topic echo /e_button_status
ros2 topic echo /stack_light_status
```

## Troubleshooting

### Common Issues

#### ROS nodes not starting:
```bash
# Verify ROS 2 installation
ros2 doctor

# Rebuild workspace
cd ros_ws && colcon build
```

#### Port conflicts:
```bash
sudo lsof -i :8080  # Check process using port
kill -9 <PID>       # Terminate conflicting process
```

#### HMI not updating:
- Check WebSocket connection in browser developer tools
- Verify ROS nodes are publishing data

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

---

## Key Features of This README:

1. **Structured Layout** - Clear sections with table of contents  
2. **Visual Component Map** - System architecture diagram (add your image to `docs/`)  
3. **Copy-Paste Ready** - All commands can be directly executed  
4. **Debugging Section** - Quick checks for each component  
5. **API Documentation** - Ready-to-use curl examples  
6. **Troubleshooting** - Common issues and solutions  

---