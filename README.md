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

## Running the Project

The simulation and the two control nodes are started in **three separate terminals**.

### Terminal 1 — Start Gazebo and RViz

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch bme_gazebo_sensors spawn_robot_launch.py
```

This starts the provided Gazebo/RViz simulation environment and the simulated robot.

### Terminal 2 — Start the Safety Controller

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run safety_controller safety_controller
```

The node subscribes to `/scan`, processes the laser data, and publishes obstacle information on `/obstacle_info`.

### Terminal 3 — Start the Robot Controller

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run robot_controller robot_controller
```

The terminal will display:

```text
Enter [linear angular] velocities:
```

Enter the desired linear and angular velocity, for example:

```text
1.0 0.0
```

This commands:

- Linear velocity = `1.0 m/s`
- Angular velocity = `0.0 rad/s`

A rotational command can be entered as:

```text
0.0 0.5
```

A stop command is:

```text
0.0 0.0
```

## Checking Topics

List the active ROS 2 topics:

```bash
ros2 topic list
```

Check the laser scanner:

```bash
ros2 topic echo /scan
```

Check the custom obstacle information:

```bash
ros2 topic echo /obstacle_info
```

Check the velocity commands sent to the robot:

```bash
ros2 topic echo /cmd_vel
```

To inspect the topic types:

```bash
ros2 topic type /scan
ros2 topic type /obstacle_info
ros2 topic type /cmd_vel
```

Expected types are:

```text
/scan              sensor_msgs/msg/LaserScan
/obstacle_info     custom_interfaces/msg/ObstacleInfo
/cmd_vel           geometry_msgs/msg/Twist
```

## Checking Services

List the available services:

```bash
ros2 service list
```

The project provides:

```text
/get_average_velocity
/set_threshold
```

Check the service types:

```bash
ros2 service type /set_threshold
ros2 service type /get_average_velocity
```

The expected types are:

```text
/set_threshold             custom_interfaces/srv/SetThreshold
/get_average_velocity      custom_interfaces/srv/GetAverageVelocity
```

## Changing the Safety Threshold

The default safety threshold is `1.0 m`.

The threshold can be changed at runtime using the `/set_threshold` service.

For example, to set the threshold to `0.5 m`:

```bash
ros2 service call /set_threshold custom_interfaces/srv/SetThreshold "{threshold: 0.5}"
```

Expected response:

```text
success: true
message: Threshold updated successfully.
```

The new threshold is then included in the published `ObstacleInfo` message.

## Checking Average Velocity

The robot controller stores the **five most recent user velocity inputs**.

After entering several velocity commands, call:

```bash
ros2 service call /get_average_velocity custom_interfaces/srv/GetAverageVelocity "{}"
```

The response contains:

```text
average_linear: ...
average_angular: ...
```

The returned values represent the average linear and angular velocity of the five most recent inputs. If fewer than five inputs have been entered, the available inputs are used.

## Testing Obstacle Detection

With the simulation and both controllers running:

1. Observe `/scan` to confirm laser data is being received.
2. Observe `/obstacle_info` to see the closest valid obstacle distance and its direction.
3. Move the robot toward an obstacle using the velocity input in Terminal 3.
4. When the closest obstacle distance becomes smaller than the configured threshold while the robot has a non-zero linear velocity, the safety mechanism is activated.

## Testing Safety Recovery

When the safety condition is triggered:

1. `safety_controller` detects that the minimum valid laser distance is below the threshold.
2. It publishes the corresponding `ObstacleInfo` message.
3. `robot_controller` receives the obstacle information.
4. The robot performs one backward movement using the opposite of its current linear velocity.
5. The robot then stops.
6. A new non-zero user velocity command allows the robot to resume operation.

Pure rotational commands are allowed even when the nearest obstacle is below the threshold.


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
