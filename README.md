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