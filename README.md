# Basic ROS2 setup with Gazebo, PX4-Autopilot and QGroundControl for drone simulation

# ROS2 PX4 Drone Control - Complete Setup Guide

## Table of Contents
1. [Prerequisites Installation](#prerequisites-installation)
2. [Environment Configuration](#environment-configuration)
3. [Project Structure Setup](#project-structure-setup)
4. [Building the Workspace](#building-the-workspace)
5. [Running the Simulation](#running-the-simulation)
6. [Monitoring and Debugging](#monitoring-and-debugging)

---

## Prerequisites Installation

### 1. ROS2 Jazzy Installation

**Update system:**
```bash
sudo apt update
sudo apt upgrade -y
```

**Add ROS2 repository:**
```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Install ROS2 Jazzy:**
```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
sudo apt install ros-dev-tools -y
```

**Setup environment:**
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify installation:**
```bash
ros2 --version
```

### 2. PX4 Autopilot Installation

**Install dependencies:**
```bash
sudo apt install git python3-pip python3-dev python3-venv -y
```

**Clone PX4 repository:**
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

**Run setup script:**
```bash
bash ./Tools/setup/ubuntu.sh
```

**Install Python dependencies:**
```bash
pip3 install --user -r Tools/setup/requirements.txt
```

**Build PX4 for SITL:**
```bash
make px4_sitl_default
```

**Test PX4 build:**
```bash
make px4_sitl gz_x500
```
Press `Ctrl+C` to stop after verification.

### 3. Gazebo Harmonic Installation

**Add Gazebo repository:**
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

**Install Gazebo Harmonic:**
```bash
sudo apt update
sudo apt install gz-harmonic -y
```

**Verify installation:**
```bash
gz sim --version
```

### 4. QGroundControl Installation

**Download QGroundControl:**
```bash
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

**Move to applications (optional):**
```bash
sudo mkdir -p /opt/qgroundcontrol
sudo mv QGroundControl.AppImage /opt/qgroundcontrol/
```

**Create desktop entry (optional):**
```bash
cat << EOF > ~/.local/share/applications/qgroundcontrol.desktop
[Desktop Entry]
Type=Application
Name=QGroundControl
Exec=/opt/qgroundcontrol/QGroundControl.AppImage
Icon=qgroundcontrol
Terminal=false
Categories=Utility;
EOF
```

### 5. Micro XRCE-DDS Agent Installation

**Install via snap:**
```bash
sudo snap install micro-xrce-dds-agent --edge
```

**Verify installation:**
```bash
micro-xrce-dds-agent --version
```

---

## Environment Configuration

### 1. Setup PX4 Environment Variables

**Create PX4 environment script:**
```bash
nano ~/px4_env.sh
```

**Add the following content:**
```bash
#!/bin/bash
cd ~/PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gz
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/Tools/simulation/gz/worlds
cd -
```

**Make it executable:**
```bash
chmod +x ~/px4_env.sh
```

**Add to bashrc:**
```bash
echo "source ~/px4_env.sh" >> ~/.bashrc
source ~/.bashrc
```

### 2. Configure ROS2 Domain ID (Optional)

```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

---

## Project Structure Setup

### 1. Create ROS2 Workspace

```bash
mkdir -p ~/ROS2/ros2_drone/src
cd ~/ROS2/ros2_drone
```

### 2. Clone PX4 ROS2 Messages

```bash
cd ~/ROS2/ros2_drone/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
```

### 3. Create Drone Controller Package

```bash
cd ~/ROS2/ros2_drone/src
ros2 pkg create --build-type ament_python drone_controller --dependencies rclpy
```

### 4. Setup Package Structure

```bash
cd ~/ROS2/ros2_drone/src/drone_controller
```

**Expected directory structure:**
```
drone_controller/
├── drone_controller/
│   ├── __init__.py
│   └── offboard_control.py
├── resource/
│   └── drone_controller
├── test/
├── package.xml
├── setup.cfg
└── setup.py
```

### 5. Configure package.xml

**Edit the file:**
```bash
nano ~/ROS2/ros2_drone/src/drone_controller/package.xml
```

**Ensure it includes:**
```xml
<depend>rclpy</depend>
<depend>px4_msgs</depend>
```

### 6. Configure setup.py

**Edit the file:**
```bash
nano ~/ROS2/ros2_drone/src/drone_controller/setup.py
```

**Add entry point in the `entry_points` section:**
```python
entry_points={
    'console_scripts': [
        'offboard_control = drone_controller.offboard_control:main',
    ],
},
```

### 7. Create Python Script

```bash
touch ~/ROS2/ros2_drone/src/drone_controller/drone_controller/offboard_control.py
chmod +x ~/ROS2/ros2_drone/src/drone_controller/drone_controller/offboard_control.py
```

*[Add your Python code here]*

---

## Building the Workspace

### 1. Install Dependencies

```bash
cd ~/ROS2/ros2_drone
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build Workspace

**First build (PX4 messages):**
```bash
cd ~/ROS2/ros2_drone
colcon build --packages-select px4_msgs
source install/setup.bash
```

**Build drone controller:**
```bash
colcon build --packages-select drone_controller
source install/setup.bash
```

**Build all packages:**
```bash
colcon build
source install/setup.bash
```

### 3. Add Workspace to bashrc (Optional)

```bash
echo "source ~/ROS2/ros2_drone/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Verify Build

```bash
ros2 pkg list | grep drone_controller
ros2 run drone_controller offboard_control --help
```

---

## Running the Simulation

### Complete Workflow

You'll need **4 separate terminal windows** for a complete setup.

#### Terminal 1: Launch PX4 SITL with Gazebo

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

**What this does:**
- Starts PX4 Software-In-The-Loop simulator
- Launches Gazebo Harmonic with X500 quadcopter model
- Creates virtual flight controller

**Expected output:**
- Gazebo window opens with drone model
- PX4 shell prompt appears
- Messages about initialization

#### Terminal 2: Start Micro XRCE-DDS Agent

```bash
micro-xrce-dds-agent udp4 -p 8888
```

**What this does:**
- Creates bridge between PX4 and ROS2
- Enables DDS communication
- Required for ROS2 topic publishing/subscribing

**Expected output:**
```
[1701234567.123456] info     | UDPv4AgentLinux.cpp | init | running...
[1701234567.123789] info     | Root.cpp           | set_verbose_level | logger setup...
```

#### Terminal 3: Launch QGroundControl (Optional)

```bash
/opt/qgroundcontrol/QGroundControl.AppImage
# OR from Downloads
~/Downloads/QGroundControl.AppImage
```

**What this does:**
- Provides GUI for monitoring
- Shows telemetry data
- Allows manual control if needed
- Displays flight path

**Features:**
- Auto-connects to PX4
- Real-time position tracking
- Battery status
- Flight mode display

#### Terminal 4: Run ROS2 Controller

**First, verify ROS2 topics are available:**
```bash
source ~/ROS2/ros2_drone/install/setup.bash
ros2 topic list
```

**Expected topics:**
```
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
```

**Run the controller:**
```bash
source ~/ROS2/ros2_drone/install/setup.bash
ros2 run drone_controller offboard_control
```

**Flight sequence:**
1. Pre-flight checks (1 second)
2. Engage offboard mode
3. Arm motors
4. Takeoff to 5 meters
5. Fly square pattern (4 waypoints)
6. Return to home
7. Descend to 1 meter
8. Land
9. Disarm

---

## Monitoring and Debugging

### Monitor ROS2 Topics

**List all topics:**
```bash
ros2 topic list
```

**Echo specific topic:**
```bash
ros2 topic echo /fmu/out/vehicle_local_position
ros2 topic echo /fmu/out/vehicle_status
```

**Check topic frequency:**
```bash
ros2 topic hz /fmu/out/vehicle_local_position
```

**View topic info:**
```bash
ros2 topic info /fmu/in/trajectory_setpoint
```

### Check ROS2 Node Status

```bash
ros2 node list
ros2 node info /offboard_control
```

### View PX4 Status

**In PX4 console (Terminal 1):**
```bash
commander status
listener vehicle_local_position
listener vehicle_status
```

---

**Last Updated**: December 2024  
**Tested On**: Ubuntu 24.04, ROS2 Jazzy, PX4 v1.14, Gazebo Harmonic
