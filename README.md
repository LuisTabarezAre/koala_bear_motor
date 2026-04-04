# Koala BEAR Motor - ROS 2 Hardware Interface

A `ros2_control` plugin (`SystemInterface`) designed for the integration, control, and state reading of BEAR actuators (Westwood Robotics) in ROS 2. This package enables high-speed RS485 communication and the safe handling of motors in advanced robotics applications.

## 🚀 Key Features

* **High-Speed Communication:** Optimized to operate at **7.5 Mbps / 8 Mbps** using `BulkRead` and `BulkWrite` operations from the BEAR SDK, minimizing bus latency.
* **Dynamic Controller Transition:** Implements `perform_command_mode_switch`, allowing real-time switching between position control (`JointTrajectoryController`) and direct torque control (`effort_controllers`).
* **Integrated Safety Protocol:** Complies with the strict hardware rule for BEAR actuators: temporarily disabling torque before any operational mode change to prevent firmware lockups.
* **Startup Synchronization:** Reads the initial state of the motors in the `on_activate` method to prevent sudden jumps or "whiplash" when starting trajectory controllers.

## 📋 System Requirements

* **Operating System:** Ubuntu 24.04 (Recommended).
* **ROS 2:** Jazzy Jalisco.
* **ROS 2 Dependencies:**
  * `rclcpp`, `rclcpp_lifecycle`
  * `hardware_interface`
  * `pluginlib`
  * `ros2_control`, `ros2_controllers`
* **External Dependencies:** Westwood Robotics C++ SDK (`CBEAR`).

## 📁 Project Structure

<pre> <code> 
bear_hardware_interface
    ├── bear_hardware_plugins.xml
    ├── CMakeLists.txt
    ├── config
    │   └── bear_controllers.yaml
    ├── description
    │   └── koala.urdf.xacro
    ├── external
    │   └── CBEAR
    │       ├── include
    │       │   ├── bear_macro.h
    │       │   ├── bear_sdk.h
    │       │   ├── exampleConfig.h.in
    │       │   ├── packet_manager.h
    │       │   └── port_manager.h
    │       └── src
    │           ├── bear_sdk.cpp
    │           ├── packet_manager.cpp
    │           ├── port_manager.cpp
    │           └── utils
    │               └── loop_time_stats.cpp
    ├── include
    │   └── bear_hardware_interface
    │       └── bear_system.hpp
    ├── launch
    │   └── bear_hardware.launch.py
    ├── package.xml
    └── src
        └── bear_system.cpp
</code> </pre>
---

## 🛠️ Installation and Build

### 1. Create the ROS 2 workspace

```bash
mkdir -p ~/ros2_ws_koala/src
cd ~/ros2_ws_koala/src
```

### 2. Clone the repository
```bash
git clone https://github.com/LuisTabarezAre/koala_bear_motor.git
```
### 3. Initialize and update the ROS dependency tool:
```bash
cd ~/ros2_ws_koala
sudo rosdep init
rosdep update
```

### 3.  Install dependencies and build the package:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bear_hardware_interface
```

### 4. Source the environment:
```bash
source install/setup.bash
```

## 🔌 Physical Layer (Hardware) Considerations

Since this project operates at frequencies up to 8 Mbps, RS485 signal integrity is critical to avoid data packet loss (failed BulkRead operations).

    Mandatory Topology: A strict Daisy-chain configuration must be used. Avoid star topologies (Fork-chain), as stubs generate destructive signal reflections at these speeds.

    Bus Termination: It is essential to place a 120 Ω termination resistor on the last physical motor in the chain.

    Grounding: Ensure that the logic GND of the motors is tied together with the GND of the USB-RS485 adapter to prevent common-mode noise issues.

## 💻 URDF Configuration

To use this hardware interface in your robot description (.xacro / .urdf), add the following block inside the <robot> tag:
XML

```bash
<ros2_control name="BearHardware" type="system">
  <hardware>
    <plugin>bear_hardware_interface/BearSystemHardware</plugin>
    <param name="port_name">/dev/ttyUSB0</param>
    <param name="baudrate">7500000</param>
    <param name="motor_ids">1,2</param>
  </hardware>

  <joint name="joint_1">
    <param name="motor_id">1</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint_2">
    <param name="motor_id">2</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## 🕹️ Basic Usage

Once the controller is running via your launch.py file, ensure both the joint_state_broadcaster and your desired controller are active.

### Check controller status:
```bash
ros2 control list_controllers
```

### Send a simple trajectory goal (Example with JointTrajectoryController):
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint_1', 'joint_2'],
    points: [
      { 
        positions: [0.5, -0.5], 
        time_from_start: { sec: 2, nanosec: 0 } 
      }
    ]
  }
}"
```
