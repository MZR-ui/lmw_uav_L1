# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS (Robot Operating System) catkin workspace for a UAV (drone) control system. The system integrates remote control input, flight controller communication via MSP protocol, gimbal control, task management, and sensor data processing.

## Build System

**Build the workspace:**
```bash
cd /home/mzr/lmw_catkin_ws/uav_ws
catkin_make
```

**Source the workspace after building:**
```bash
source devel/setup.bash
```

**Build a single package:**
```bash
catkin_make --pkg <package_name>
```

**Clean build:**
```bash
catkin_make clean
catkin_make
```

## Running the System

**Launch all nodes:**
```bash
roslaunch uav_bringup uav_all.launch
```

**Launch individual nodes:**
```bash
roslaunch remote_info remote_info.launch
roslaunch msp_interface msp_interface.launch
roslaunch gimbal_control_serial gimbal_control.launch
roslaunch task_manager task_manager.launch
roslaunch subtask subtask.launch
roslaunch sensing realsense.launch
```

**Monitor topics:**
```bash
rostopic list                    # List all topics
rostopic echo /remote_order      # Monitor remote control data
rostopic echo /order_task        # Monitor task commands
rostopic echo /control_data      # Monitor control output
rostopic echo /gimbal/status     # Monitor gimbal status
```

**Check node status:**
```bash
rosnode list                     # List running nodes
rosnode info <node_name>         # Get node details
```

## System Architecture

### Data Flow

The system follows this primary data flow:

1. **Remote Control Input** → `remote_info` node reads IBUS protocol from RC receiver ��� publishes `/remote_order` (Remote.msg)

2. **Task Management** �� `task_manager` node subscribes to `/remote_order` → maps RC channel values to task IDs → publishes `/order_task` (OrderTask.msg)

3. **Task Execution** → `subtask` node (Task1BaseAct) subscribes to `/order_task` → generates control commands → publishes `/control_data` (ControlData.msg)

4. **Flight Controller** → `msp_interface` nodes communicate with flight controller via MSP V2 protocol → publish sensor data (Attitude, MspSensor, EscTelem, MspChannel)

5. **Gimbal Control** → `gimbal_control_serial` subscribes to `/gimbal/cmd` ��� controls gimbal via serial → publishes `/gimbal/status`

### Package Descriptions

- **remote_info**: Parses IBUS protocol from RC receiver (14 channels, 1000-2000 PWM range)
- **msp_interface**: MSP V2 protocol communication with flight controller (includes CRC8/CRC16 validation)
- **task_manager**: Maps RC channel values to discrete task IDs (0=idle, 1=task1, 2=task2)
- **subtask**: Executes tasks by generating control commands (throttle, pitch, roll, yaw, gimbal)
- **gimbal_control_serial**: Serial communication with gimbal hardware
- **sensing**: RealSense camera integration (currently disabled in main launch)
- **uav_bringup**: Main launch files for system startup

### Serial Communication

All serial communication uses 115200 baud rate:
- **IBUS protocol**: Remote control receiver (remote_info)
- **MSP V2 protocol**: Flight controller (msp_interface)
- **Custom protocol**: Gimbal control (gimbal_control_serial)

Serial port configuration is in `config/*.yaml` files in each package.

### Key Message Types

- `Remote.msg`: 14-channel RC data array
- `OrderTask.msg`: Task ID (uint8)
- `ControlData.msg`: Throttle, pitch, roll, yaw + gimbal control (mode, pitch, roll, yaw)
- `GimbalCmd.msg` / `GimbalStatus.msg`: Gimbal command and status
- `Attitude.msg`: Roll, pitch, yaw + quaternion orientation
- `MspSensor.msg`: Flight controller sensor data
- `MspChannel.msg`: Flight controller RC channels
- `EscTelem.msg`: ESC telemetry data

## Code Conventions

- **Language**: C++ (ROS nodes), following ROS C++ style guide
- **Namespaces**: Each package uses its own namespace matching the package name
- **Node structure**: Class-based nodes with constructor taking `ros::NodeHandle` references
- **Thread safety**: Uses `std::mutex` for shared data between callbacks
- **Parameters**: Loaded via `ros::NodeHandle::param()` from YAML config files
- **Logging**: Uses ROS logging macros (ROS_INFO, ROS_DEBUG, ROS_ERROR)

## Configuration Files

Configuration files are in `<package>/config/*.yaml`:
- Serial port paths and baud rates
- Node-specific parameters (thresholds, rates, channel mappings)
- Load via `<rosparam command="load" file="..."/>` in launch files

## Development Notes

- The workspace is located at `/home/mzr/lmw_catkin_ws/uav_ws`
- This is a WSL2 environment (Ubuntu 20.04) accessed from Windows
- ROS distribution appears to be ROS Noetic (based on package format 2)
- The `sensing` package (RealSense) is currently commented out in the main launch file
- Serial device paths may need adjustment based on hardware connections

## Adding New Packages

When creating new packages:
```bash
cd src
catkin_create_pkg <package_name> roscpp std_msgs [other_dependencies]
cd ..
catkin_make
```

## Debugging

**Check for build errors:**
```bash
catkin_make 2>&1 | tee build.log
```

**Monitor all topics simultaneously:**
```bash
rqt_graph          # Visualize node/topic graph
rqt_console        # View log messages
```

**Test serial connections:**
```bash
ls -l /dev/ttyUSB*  # List USB serial devices
```
