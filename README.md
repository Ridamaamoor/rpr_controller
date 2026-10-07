# ROS 2 Obstacle-Aware Robot Controller

A ROS 2 implementation of an obstacle-aware control system for a simulated mobile robot. The system combines user-controlled robot motion, laser-based obstacle detection, safety recovery, custom ROS 2 interfaces, and velocity statistics.

## System Architecture

```text
                         Gazebo Robot
                              │
                           /scan
                              │
                              ▼
                  ┌──────────────────────┐
                  │  safety_controller   │
                  │                      │
                  │  Laser processing    │
                  │  Minimum distance    │
                  │  Obstacle direction  │
                  └──────────┬───────────┘
                             │
                      /obstacle_info
                             │
                             ▼
                  ┌──────────────────────┐
                  │   robot_controller   │
                  │                      │
                  │  User velocity input │
                  │  Safety recovery     │
                  │  Velocity history    │
                  └──────────┬───────────┘
                             │
                          /cmd_vel
                             │
                             ▼
                         Gazebo Robot
```

The two nodes communicate through ROS 2 topics, while services provide runtime configuration and velocity statistics.

## Packages

### `custom_interfaces`

Contains the custom ROS 2 interfaces used by the system:

- **`ObstacleInfo.msg`** — closest obstacle distance, direction and current threshold.
- **`SetThreshold.srv`** — changes the safety threshold at runtime.
- **`GetAverageVelocity.srv`** — returns the average linear and angular velocity of the five most recent user inputs.

### `safety_controller`

Processes the robot's laser scanner data, finds the closest valid obstacle and determines its direction. The resulting information is published through the custom `ObstacleInfo` message.

### `robot_controller`

Accepts user-defined linear and angular velocities and publishes them to the robot. It also monitors obstacle information and performs the safety recovery when required, while maintaining a history of the five most recent velocity inputs.

## Communication

| Node | Interface | Type | Purpose |
|---|---|---|---|
| `safety_controller` | `/scan` | `LaserScan` | Laser data |
| `safety_controller` | `/obstacle_info` | `ObstacleInfo` | Obstacle information |
| `safety_controller` | `/set_threshold` | Service | Runtime threshold configuration |
| `robot_controller` | `/cmd_vel` | `Twist` | Robot velocity |
| `robot_controller` | `/obstacle_info` | `ObstacleInfo` | Safety information |
| `robot_controller` | `/get_average_velocity` | Service | Velocity statistics |

## Safety Logic

The system uses the minimum valid laser distance as the safety indicator.

When the detected distance falls below the configured threshold **and the robot has a non-zero linear velocity**, the robot performs a single backward recovery and then stops.

Pure rotational commands are allowed.

## Simulation Environment

The project uses the **`bme_gazebo_sensors`** simulation environment developed by Carmine D8:

https://github.com/CarmineD8/bme_gazebo_sensors

The repository provides the Gazebo simulation environment, including the robot model, sensors, worlds, RViz configuration and launch resources.

## Requirements

- ROS 2 Jazzy
- Gazebo / ROS-Gazebo integration
- `bme_gazebo_sensors` simulation package

## Build

From the ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select custom_interfaces safety_controller robot_controller
source install/setup.bash
```

## Run

The simulation environment and the two control nodes are launched separately:

```text
Terminal 1 → Gazebo / RViz simulation
Terminal 2 → safety_controller
Terminal 3 → robot_controller
```

The robot controller provides interactive velocity input:

```text
Enter [linear angular] velocities:
```

## Project Structure

```text
ros2_ws/src/
│
├── custom_interfaces/
│   ├── msg/
│   │   └── ObstacleInfo.msg
│   ├── srv/
│   │   ├── SetThreshold.srv
│   │   └── GetAverageVelocity.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
├── robot_controller/
│   ├── src/
│   │   └── robot_controller.cpp
│   ├── CMakeLists.txt
│   └── package.xml
│
└── safety_controller/
    ├── src/
    │   └── safety_controller.cpp
    ├── CMakeLists.txt
    └── package.xml
```
