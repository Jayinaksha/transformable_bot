# Transformable Bot 🔄🚁🚗

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/)
[![Build](https://github.com/Jayinaksha/transformable_bot/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/Jayinaksha/transformable_bot/actions)

A ROS 2 quadcopter robot with **dynamic mode switching** — transform between **drone mode** (rigid arms locked for stable flight) and **rover mode** (arms free for ground movement) at runtime, without restarting the simulation.

---

## 🎯 Features

- **🔄 Dynamic Mode Switching** — Toggle each arm between rigid (drone) and free (rover) configurations via a ROS 2 service call, live in simulation
- **🚁 Drone Mode** — Each arm's ring assembly is rigidly coupled to the base, enabling stable flight posture
- **🚗 Rover Mode** — Each arm's motor assembly decouples from the base, allowing ground articulation
- **⚙️ 2-DOF Servo Gimbal** — Each arm has an X-axis and Y-axis servo controlled by PD controllers for independent motor orientation
- **🔧 Motor Control** — Individual brushless motor speed control per arm (velocity command in rad/s, up to ±2000 rad/s)
- **💍 Propeller Guard Ring System** — Each propeller has a 90-segment ring guard attached via 9 radial rods
- **🔌 Modular Plugin Architecture** — Four custom Gazebo plugins built as shared libraries with ROS 2 node integration
- **📡 ROS 2 / Gazebo Harmonic** — Built with `gz-sim8`, `gz-plugin2`, `gz-common5`, and `rclcpp`

---

## 🏗️ Robot Architecture

### Physical Structure

```
base_link (0.55 × 0.55 × 0.12 m body)
├── front_right arm  (45°)
│   ├── front_right_y_servo  → front_right_arm
│   │   └── front_right_x_servo → front_right_gimbal_base
│   │       └── front_right_extension → front_right_motor_hub
│   │           └── front_right_motor  (continuous joint, plugin-controlled)
│   │               └── propeller_front_right (2-blade, fixed to motor)
│   │                   └── ring_attach_front_right_hub_9rods
│   │                       ├── 9 radial rods (angled outward)
│   │                       └── 90-segment ring guard (r = 0.18 m)
├── back_right arm   (135°)  — same sub-tree as above
├── back_left arm    (225°)  — same sub-tree as above
└── front_left arm   (315°)  — same sub-tree as above
```

### Joints

| Joint | Type | Description |
|-------|------|-------------|
| `base_to_{arm}_y_servo` | fixed | Attaches Y-servo to base at arm angle |
| `{arm}_y_servo_to_arm` | revolute (Y) | Y-axis tilt (0 – 90°) |
| `{arm_link}_to_{arm}_x_servo` | fixed | Attaches X-servo to arm tip |
| `{arm}_x_servo_to_gimbal` | revolute (Z) | X-axis pan (±90°) |
| `{arm}_gimbal_to_extension` | fixed | Gimbal to mini-arm extension |
| `{arm}_extension_to_motor_hub` | fixed | Extension to motor hub |
| `{arm}_motor_hub_to_motor` | continuous | Motor spin joint (plugin-driven) |
| `{arm}_motor_to_propeller` | fixed | Motor to propeller hub |
| `ring_attach_{arm}_hub_9rods_joint` | continuous (free-spin) | Ring+rods hub attached to propeller hub |

---

## 🔌 Gazebo Plugins

Four custom Gazebo system plugins are built and loaded at simulation start:

### 1. `dynamic_joint_mode_switch_plugin` (`libdynamic_joint_mode_switch_plugin.so`)

Dynamically creates or destroys fixed joints between robot links at runtime via ECM (Entity Component Manager) — no SDF entity creator required.

- **Drone Mode** (`data: true`) — Creates a fixed joint between `base_link` and `ring_attach_{arm}_hub_9rods` (rigid flight structure)
- **Rover Mode** (`data: false`) — Creates a fixed joint between `ring_attach_{arm}_hub_9rods` and `{arm}_motor` (arm pivot point moves)
- On each mode switch, the previous joint is removed and a new one is created
- Publishes the current mode string to `/robot/{arm}/current_mode`

**SDF Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arm_name` | `arm_N` | Name of the arm (e.g. `front_right`) |
| `service_name` | `/robot/{arm}/mode_switch` | ROS 2 service name |

### 2. `motor_plugin` (`libmotor_plugin.so`)

Controls the continuous spin joint `{arm}_motor_hub_to_motor` via `JointVelocityCmd`.

- Subscribes to a velocity topic (rad/s), applies ±2000 rad/s clamp, and sets the joint velocity each simulation step
- Searches for the target joint by name during early simulation frames; retries every 100 ms up to 100 attempts

**SDF Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arm_name` | `arm_N` | Name of the arm |
| `joint_name` | `{arm}_motor_hub_to_motor` | Joint to drive |
| `velocity_topic` | `/robot/{arm}/motor/velocity` | ROS 2 topic |
| `max_velocity` | `2000.0` (rad/s) | Velocity clamp |

### 3. `servo_y_plugin` (`libservo_y_plugin.so`)

PD position controller for the Y-axis arm tilt joint `{arm}_y_servo_to_arm`.

- Subscribes to a position topic (radians), clamps to ±1.57 rad (±90°), applies PD torque via `JointForceCmd`
- Default gains: Kp = 1000, Kd = 100, max force = 100 N·m

**SDF Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arm_name` | `arm_N` | Name of the arm |
| `joint_name` | `{arm}_y_servo_to_arm` | Joint to control |
| `position_topic` | `/robot/{arm}/y_servo/position` | ROS 2 topic |
| `kp` | `1000.0` | Proportional gain |
| `kd` | `100.0` | Derivative gain |
| `max_force` | `100.0` | Force clamp (N·m) |

### 4. `servo_x_plugin` (`libservo_x_plugin.so`)

PD position controller for the X-axis gimbal pan joint `{arm}_x_servo_to_gimbal`.

- Same architecture as `servo_y_plugin`, but controls the X-axis revolute joint
- Default gains: Kp = 1000, Kd = 100, max force = 100 N·m

**SDF Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `arm_name` | `arm_N` | Name of the arm |
| `joint_name` | `{arm}_x_servo_to_gimbal` | Joint to control |
| `position_topic` | `/robot/{arm}/x_servo/position` | ROS 2 topic |
| `kp` | `1000.0` | Proportional gain |
| `kd` | `100.0` | Derivative gain |
| `max_force` | `100.0` | Force clamp (N·m) |

---

## 📡 ROS 2 Interface

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/robot/{arm}/mode_switch` | `std_srvs/srv/SetBool` | `true` = drone mode, `false` = rover mode |

### Topics (subscribed)

| Topic | Type | Description |
|-------|------|-------------|
| `/robot/{arm}/motor/velocity` | `std_msgs/msg/Float64` | Motor speed in rad/s (clamped to ±2000) |
| `/robot/{arm}/y_servo/position` | `std_msgs/msg/Float64` | Y-servo target angle in radians (±1.57) |
| `/robot/{arm}/x_servo/position` | `std_msgs/msg/Float64` | X-servo target angle in radians (±1.57) |

### Topics (published)

| Topic | Type | Description |
|-------|------|-------------|
| `/robot/{arm}/current_mode` | `std_msgs/msg/String` | `"drone"` or `"rover"` |

> `{arm}` is one of: `front_right`, `front_left`, `back_right`, `back_left`

---

## 🚀 Quick Start

### Prerequisites

- **ROS 2 Jazzy** — [Install guide](https://docs.ros.org/en/jazzy/Installation.html)
- **Gazebo Harmonic** — [Install guide](https://gazebosim.org/docs/harmonic/install)
- **ros_gz** bridge package: `sudo apt install ros-jazzy-ros-gz`
- **xacro**: `sudo apt install ros-jazzy-xacro`
- **robot_state_publisher**: `sudo apt install ros-jazzy-robot-state-publisher`

### Installation

```bash
# Create workspace
mkdir -p ~/transformable_bot_ws/src
cd ~/transformable_bot_ws/src

# Clone repository
git clone https://github.com/Jayinaksha/transformable_bot.git

# Install ROS 2 dependencies
cd ~/transformable_bot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select transformable_bot
source install/setup.bash
```

### Launch Simulation

```bash
ros2 launch transformable_bot view_robot.launch.py
```

This will:
1. Start `robot_state_publisher` with the URDF generated from xacro
2. Launch Gazebo Harmonic (`gz sim -v4 empty.sdf`)
3. Spawn the robot at `(0, 0, 1)` metres

> **Note:** The launch file automatically sets `GZ_SIM_SYSTEM_PLUGIN_PATH` to the install directory so all four plugins are found.

---

## 🎮 Usage

### Mode Switching

```bash
# Switch all arms to ROVER mode (arms can articulate freely)
ros2 service call /robot/front_right/mode_switch std_srvs/srv/SetBool "{data: false}"
ros2 service call /robot/front_left/mode_switch  std_srvs/srv/SetBool "{data: false}"
ros2 service call /robot/back_right/mode_switch  std_srvs/srv/SetBool "{data: false}"
ros2 service call /robot/back_left/mode_switch   std_srvs/srv/SetBool "{data: false}"

# Switch all arms to DRONE mode (arms rigid for flight)
ros2 service call /robot/front_right/mode_switch std_srvs/srv/SetBool "{data: true}"
ros2 service call /robot/front_left/mode_switch  std_srvs/srv/SetBool "{data: true}"
ros2 service call /robot/back_right/mode_switch  std_srvs/srv/SetBool "{data: true}"
ros2 service call /robot/back_left/mode_switch   std_srvs/srv/SetBool "{data: true}"

# Monitor current mode for each arm
ros2 topic echo /robot/front_right/current_mode
```

### Motor Control

Velocity is in **rad/s**. Maximum is ±2000 rad/s.

```bash
# Spin all motors at 50 rad/s
ros2 topic pub /robot/front_right/motor/velocity std_msgs/msg/Float64 "{data: 50.0}" --once
ros2 topic pub /robot/front_left/motor/velocity  std_msgs/msg/Float64 "{data: 50.0}" --once
ros2 topic pub /robot/back_right/motor/velocity  std_msgs/msg/Float64 "{data: 50.0}" --once
ros2 topic pub /robot/back_left/motor/velocity   std_msgs/msg/Float64 "{data: 50.0}" --once

# Stop all motors
ros2 topic pub /robot/front_right/motor/velocity std_msgs/msg/Float64 "{data: 0.0}" --once
ros2 topic pub /robot/front_left/motor/velocity  std_msgs/msg/Float64 "{data: 0.0}" --once
ros2 topic pub /robot/back_right/motor/velocity  std_msgs/msg/Float64 "{data: 0.0}" --once
ros2 topic pub /robot/back_left/motor/velocity   std_msgs/msg/Float64 "{data: 0.0}" --once
```

### Servo Gimbal Control

Angles are in **radians**, clamped to ±1.57 rad (±90°).

```bash
# Tilt front-right arm upward 45° on Y axis
ros2 topic pub /robot/front_right/y_servo/position std_msgs/msg/Float64 "{data: 0.785}" --once

# Pan front-right gimbal 30° on X axis
ros2 topic pub /robot/front_right/x_servo/position std_msgs/msg/Float64 "{data: 0.524}" --once

# Return all servos to neutral
ros2 topic pub /robot/front_right/y_servo/position std_msgs/msg/Float64 "{data: 0.0}" --once
ros2 topic pub /robot/front_right/x_servo/position std_msgs/msg/Float64 "{data: 0.0}" --once
```

---

## 📦 Package Structure

```
transformable_bot/
├── CMakeLists.txt                    # Build configuration (4 shared-library targets)
├── package.xml                       # ROS 2 package manifest
├── LICENSE                           # License file
├── README.md                         # This file
├── launch/
│   └── view_robot.launch.py          # Main launch: Gazebo + robot_state_publisher + spawn
├── urdf/
│   ├── transformable_bot.urdf.xacro  # Top-level robot definition (body, arms, propellers, rings)
│   ├── ring_system.urdf.xacro        # 90-segment ring guard macro
│   ├── ring_attached.urdf.xacro      # Ring + 9-rod assembly macro (attached per propeller)
│   ├── mode_switch_plugin.urdf.xacro # Gazebo plugin tags for dynamic joint mode switch
│   ├── servo_motor_plugins.urdf.xacro# Gazebo plugin tags for servo X/Y and motor plugins
│   ├── d455_.urdf.xacro              # Placeholder for depth camera (Intel D455)
│   └── transformable_bot.urdf        # Pre-generated URDF (reference copy)
├── src/
│   ├── dynamic_joint_mode_switch_plugin.cpp  # Mode switch plugin (drone ↔ rover)
│   ├── motor_plugin.cpp                       # Motor velocity control plugin
│   ├── servo_x_plugin.cpp                     # X-axis gimbal PD servo plugin
│   └── servo_y_plugin.cpp                     # Y-axis arm PD servo plugin
└── config/
    └── view_robot.rviz               # RViz2 configuration
```

---

## 🔧 Build Details

### Dependencies (`package.xml`)

| Package | Role |
|---------|------|
| `ament_cmake` | Build system |
| `rclcpp` | ROS 2 C++ client library |
| `std_msgs` | `Float64`, `String` messages |
| `std_srvs` | `SetBool` service |
| `gz-sim8` | Gazebo Harmonic simulation API |
| `gz-plugin2` | Gazebo plugin registration macros |
| `gz-common5` | Gazebo common utilities |

### Built Targets

| Library | Source file | Plugin alias(es) |
|---------|-------------|-----------------|
| `libdynamic_joint_mode_switch_plugin.so` | `dynamic_joint_mode_switch_plugin.cpp` | `dynamic_joint_mode_switch_plugin`, `mode_switch_plugin`, `front_right_mode_switch`, `back_right_mode_switch`, `back_left_mode_switch`, `front_left_mode_switch` |
| `libmotor_plugin.so` | `motor_plugin.cpp` | `motor_plugin`, `front_right_motor`, `front_left_motor`, `back_right_motor`, `back_left_motor` |
| `libservo_x_plugin.so` | `servo_x_plugin.cpp` | `servo_x_plugin`, `front_right_servo_x`, `front_left_servo_x`, `back_right_servo_x`, `back_left_servo_x` |
| `libservo_y_plugin.so` | `servo_y_plugin.cpp` | `servo_y_plugin`, `front_right_servo_y`, `front_left_servo_y`, `back_right_servo_y`, `back_left_servo_y` |

### Build & Test Commands

```bash
# Standard build
colcon build --packages-select transformable_bot

# Debug build
colcon build --packages-select transformable_bot --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run linting tests
colcon test --packages-select transformable_bot
colcon test-result --verbose
```

---

## 🐛 Troubleshooting

**Plugin not loading / simulation crashes at startup**
```bash
# Verify plugin path is set (the launch file does this automatically)
echo $GZ_SIM_SYSTEM_PLUGIN_PATH

# Set manually if launching outside ROS 2 launch system
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/transformable_bot_ws/install/transformable_bot/lib/transformable_bot
```

**Mode switch service returns "not ready"**

The plugin searches for robot links for up to 5 seconds (50 attempts × 100 ms). Wait until the robot is fully spawned before calling the service.
```bash
# Check that links were found (plugin logs to /rosout)
ros2 topic echo /rosout | grep "All 3 links found"
```

**Motor/servo not responding**
```bash
# List all available topics
ros2 topic list | grep robot

# Check that services are available
ros2 service list | grep mode_switch

# Inspect plugin log output
ros2 topic echo /rosout
```

**Joint search fails after 100 attempts**

The plugin will log all available joint names. Check that the URDF was processed correctly:
```bash
# Verify URDF generation
ros2 param get /robot_state_publisher robot_description | head -50
```

---

## 📊 System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | Quad-core (Intel i5 / Ryzen 5) | 8-core or better |
| RAM | 8 GB | 16 GB |
| GPU | Integrated graphics | Dedicated GPU (NVIDIA/AMD) |
| OS | Ubuntu 22.04 (Jammy) | Ubuntu 24.04 (Noble) |
| ROS 2 | Jazzy | Jazzy |
| Gazebo | Harmonic | Harmonic |

---

## 🤝 Contributing

1. **Fork** this repository
2. **Create** a feature branch: `git checkout -b feature/your-feature`
3. **Commit** your changes: `git commit -m 'feat: add your feature'`
4. **Push**: `git push origin feature/your-feature`
5. **Open** a Pull Request against `master`

**Code style:**
- C++: Follow the [ROS 2 C++ style guide](https://docs.ros.org/en/jazzy/Contributing/Code-Style-Language-Versions.html)
- Python: PEP 8
- All public plugin methods and SDF parameters should be documented in the source

---

## 📄 License

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.

---

## 📧 Contact

**Author**: Jayinaksha  
**GitHub**: [@Jayinaksha](https://github.com/Jayinaksha)  
**Repository**: [https://github.com/Jayinaksha/transformable_bot](https://github.com/Jayinaksha/transformable_bot)

---

⭐ **Star this repository if you find it helpful!** ⭐
