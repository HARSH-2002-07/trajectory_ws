# Trajectory Control System for Differential Drive Robots

A comprehensive ROS2-based trajectory tracking system implementing path smoothing, trajectory generation, and Pure Pursuit control for differential drive robots (TurtleBot3).

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Setup and Installation](#setup-and-installation)
4. [Execution Instructions](#execution-instructions)
5. [Design Choices and Algorithms](#design-choices-and-algorithms)
6. [Real Robot Deployment](#real-robot-deployment)
7. [AI Tools Used](#ai-tools-used)
8. [Obstacle Avoidance Extension](#obstacle-avoidance-extension)
9. [Testing and Quality Assurance](#testing-and-quality-assurance)
10. [Results and Performance](#results-and-performance)

---

## Overview

This project implements a complete trajectory tracking solution for differential drive robots, addressing three key challenges in mobile robotics:

### Core Features
- **Path Smoothing**: Catmull-Rom spline interpolation for C1 continuous smooth paths
- **Trajectory Generation**: Time-parameterized trajectory with configurable sampling rates
- **Trajectory Tracking**: Pure Pursuit controller with adaptive velocity control
- **Visualization**: Real-time path and robot state visualization in RViz2
- **Modular Architecture**: Clean separation of concerns with well-defined interfaces

### Performance Characteristics
- Control frequency: 20 Hz (configurable)
- Position accuracy: < 10 cm goal tolerance
- Smooth velocity profiles with adaptive speed reduction on sharp turns
- Support for multiple trajectory patterns (figure-8, spiral, race track, etc.)

---

## System Architecture

### Component Diagram
```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 System Architecture                  │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐         ┌─────────────────────────────┐  │
│  │   Waypoint   │─waypts─▶│  Trajectory Controller      │  │
│  │  Publisher   │         │                             │  │
│  └──────────────┘         │  ┌────────────────────────┐ │  │
│                           │  │   Path Smoother        │ │  │
│                           │  │  (Catmull-Rom Spline)  │ │  │
│  ┌──────────────┐         │  └────────────────────────┘ │  │
│  │   Gazebo     │         │                             │  │
│  │ Simulation   │─odom───▶│  ┌────────────────────────┐ │  │
│  │              │◀─cmd_vel─│  │  Pure Pursuit          │ │  │
│  └──────────────┘         │  │  Controller            │ │  │
│                           │  └────────────────────────┘ │  │
│  ┌──────────────┐         └─────────────────────────────┘  │
│  │    RViz2     │◀─smooth_path───────────┘                 │
│  │ Visualization│                                           │
│  └──────────────┘                                           │
└─────────────────────────────────────────────────────────────┘
```

### Software Architecture

#### 1. **PathSmoother Class**
- **Purpose**: Converts discrete waypoints into smooth, continuous trajectories
- **Algorithm**: Catmull-Rom spline interpolation
- **Key Features**:
  - C1 continuity (smooth velocity transitions)
  - Configurable sampling density
  - Handles edge cases (2 waypoints → linear interpolation)

#### 2. **PurePursuitController Class**
- **Purpose**: Generates velocity commands for trajectory following
- **Algorithm**: Pure Pursuit with adaptive velocity
- **Key Features**:
  - Dynamic lookahead distance
  - Velocity limiting for safety
  - Adaptive speed reduction on curves
  - Goal tolerance checking

#### 3. **TrajectoryControllerNode Class**
- **Purpose**: ROS2 node orchestrating the complete system
- **Responsibilities**:
  - Message handling (waypoints, odometry)
  - Timer-based control loop
  - Path visualization publishing
  - Parameter management

### Data Flow
```
Waypoints (PoseArray)
    ↓
PathSmoother::smooth_path()
    ↓
Smooth Path (vector<Point2D>)
    ↓
PurePursuitController::compute_control()
    ↓
Velocity Commands (Twist)
    ↓
Robot Motion
```

---

## Setup and Installation

### Prerequisites
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Gazebo**: Classic 11 or Ignition
- **TurtleBot3**: Package installed

### Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-tf2-geometry-msgs
```

### Package Installation

#### 1. Create Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Clone Repository
```bash
git clone <your-repository-url> trajectory_control
cd ~/ros2_ws
```

#### 3. Build Package
```bash
colcon build --packages-select trajectory_control
source install/setup.bash
```

### Package Structure
```
trajectory_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── controller_params.yaml      # Controller parameters
│   └── trajectory_control.rviz     # RViz configuration
├── launch/
│   └── trajectory_control.launch.py
├── src/
│   └── trajectory_controller_node.cpp
├── scripts/
│   └── waypoint_publisher.py
└── test/
    ├── test_path_smoother.cpp
    ├── test_pure_pursuit.cpp
    └── test_integration.cpp
```

### Environment Setup
```bash
# Add to ~/.bashrc
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## Execution Instructions

### Quick Start (All-in-One Launch)
```bash
# Launch complete system with simulation
ros2 launch trajectory_control trajectory_control.launch.py
```

This single command starts:
- Gazebo simulation with TurtleBot3
- Trajectory controller node
- RViz2 visualization
- Waypoint publisher (auto-publishes after 2s delay)

### Step-by-Step Launch (For Debugging)

#### Terminal 1: Gazebo Simulation
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

#### Terminal 2: Trajectory Controller
```bash
ros2 run trajectory_control trajectory_controller_node \
    --ros-args \
    -p lookahead_distance:=0.5 \
    -p max_linear_velocity:=0.5 \
    -p max_angular_velocity:=2.0
```

#### Terminal 3: RViz Visualization
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix trajectory_control)/share/trajectory_control/config/trajectory_control.rviz
```

#### Terminal 4: Waypoint Publisher
```bash
# Default pattern (extended_figure8)
ros2 run trajectory_control waypoint_publisher.py

# Specific pattern
ros2 run trajectory_control waypoint_publisher.py \
    --ros-args -p pattern:=spiral

# Manual publishing (no auto-publish)
ros2 run trajectory_control waypoint_publisher.py \
    --ros-args -p auto_publish:=false
```

### Available Trajectory Patterns

| Pattern | Description | Waypoints | Length |
|---------|-------------|-----------|--------|
| `extended_figure8` | Large figure-8 pattern | 25 | ~15m |
| `large_loop` | Circular path | 32 | ~19m |
| `spiral` | Expanding spiral | 48 | ~12m |
| `zigzag` | Zigzag across space | 9 | ~15m |
| `race_track` | Straights and curves | 30 | ~18m |
| `exploration` | Area coverage pattern | 17 | ~22m |
| `circle` | Simple circle | 24 | ~16m |
| `square` | Simple square | 5 | ~12m |

### Runtime Parameter Tuning
```bash
# Adjust lookahead distance (affects smoothness)
ros2 param set /trajectory_controller lookahead_distance 0.7

# Adjust maximum velocities
ros2 param set /trajectory_controller max_linear_velocity 0.3
ros2 param set /trajectory_controller max_angular_velocity 1.5
```

### Monitoring System

#### Check Topics
```bash
ros2 topic list
ros2 topic echo /smooth_path
ros2 topic hz /cmd_vel
```

#### Check Node Status
```bash
ros2 node info /trajectory_controller
```

#### View Logs
```bash
ros2 run rqt_console rqt_console
```

---
