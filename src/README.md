# Path Tracker for a 3-Wheel Swerve Robot (ROS 2 Humble)

A path-tracking system for a three-wheel swerve robot using ROS 2 Humble. The system supports path following, localization, and CAN-based motor control.

The path tracker follows a predefined path using a look-ahead tracking algorithm. It is designed to work with localization from FAST-LIO2 using a Livox MID-360 LiDAR and can control the robot through a CAN interface.

## Features

- Path tracking using a look-ahead algorithm.
- ROS 2 Humble integration.
- Localization support using FAST-LIO2 and Livox MID-360.
- CAN-based motor control.
- Planned path input using the `nav_msgs/Path` message type.
- Configurable tracking parameters.
- Support for a three-wheel swerve robot.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- `colcon`
- Required ROS 2 packages and dependencies
- FAST-LIO2 and Livox MID-360 drivers for real-robot localization
- A compatible CAN interface for real-robot motor control

## Installation

### 1. Clone the Repository

Clone the repository into your home directory:

```bash
cd ~
git clone https://github.com/OneDinTech/lbs_path_tracking_ws.git
```

Navigate to the workspace:

```bash
cd ~/lbs_path_tracking_ws
```

### 2. Source ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
```

### 3. Install Dependencies

Install the dependencies required by the packages in the workspace:

```bash
cd ~/lbs_path_tracking_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace

Build the workspace using `colcon`:

```bash
cd ~/lbs_path_tracking_ws
colcon build
```

After the build completes, source the workspace:

```bash
source ~/lbs_path_tracking_ws/install/setup.bash
```

### 5. Automatically Source the Workspace

To avoid manually sourcing ROS 2 and the workspace every time you open a new terminal, add the following lines to your `~/.bashrc` file:

```bash
source /opt/ros/humble/setup.bash
source ~/lbs_path_tracking_ws/install/setup.bash
```

Apply the changes:

```bash
source ~/.bashrc
```

**Note:** If your workspace is located somewhere other than `~/lbs_path_tracking_ws`, update the paths accordingly.

## Running the System

The following steps describe the general workflow for running the path-tracking system. The exact commands depend on the ROS 2 packages and launch files included in the repository.

### 1. Navigate to the Workspace

```bash
cd ~/lbs_path_tracking_ws
source install/setup.bash
```

### 2. Launch the Path Tracker

Run the appropriate launch file for your path-tracking package:

```bash
ros2 launch <package_name> <launch_file>.launch.py
```

Replace `<package_name>` and `<launch_file>` with the actual package and launch-file names in your workspace.

If the package does not provide a launch file, run the appropriate ROS 2 node directly:

```bash
ros2 run <package_name> <executable_name>
```

Replace the placeholders with the actual package name and executable.

### 3. Provide a Planned Path

The path tracker subscribes to a planned path published as a `nav_msgs/Path` message on the `/plan` topic.

Check whether the topic is available:

```bash
ros2 topic list
```

Inspect the topic information:

```bash
ros2 topic info /plan
```

Display the published path message:

```bash
ros2 topic echo /plan
```

Ensure that a path-planning or path-generation node publishes a valid path to `/plan` before starting path tracking.

The path must use the coordinate frame expected by the path-tracking controller.

### 4. Configure Localization

For real-robot operation, localization can be provided by FAST-LIO2 using the Livox MID-360 LiDAR.

Before starting the path tracker, ensure that:

- The Livox MID-360 is connected and publishing LiDAR data.
- FAST-LIO2 is running correctly.
- The localization system publishes the required pose or transform information.
- The coordinate frames are configured consistently.
- The path-tracking controller receives the localization information it requires.

The exact topics and coordinate frames depend on the FAST-LIO2 configuration and the implementation of the path-tracking node.

### 5. Configure Motor Control

For real-robot operation, ensure that:

- The CAN interface is configured correctly.
- The motor controller is connected and responding.
- The robot's wheel geometry and kinematic parameters are configured correctly.
- The motor commands are consistent with the robot's kinematic model.
- The emergency-stop mechanism is functional.

Test the system at low speed before performing full path-tracking experiments.

## Path Generation

The path tracker receives a planned path as input.

A planned path is typically represented by a `nav_msgs/Path` message containing a sequence of `geometry_msgs/PoseStamped` poses.

Each pose represents a position and orientation along the planned trajectory.

The path publisher should ensure that:

- Waypoints are ordered correctly.
- The path uses the expected coordinate frame.
- The path contains valid waypoints.
- The waypoint spacing is appropriate for the controller.
- The planned trajectory is suitable for the robot's kinematic constraints.
- The path is published on the topic expected by the path-tracking node.

The path tracker uses the supplied path to calculate the robot's movement commands.

## Configuration and Tuning

Path-tracking performance depends on the robot's geometry, controller parameters, localization accuracy, and the characteristics of the planned path.

Parameters that may require tuning include:

### Look-Ahead Distance

The look-ahead distance determines how far ahead of the robot the controller selects a target point on the path.

A larger look-ahead distance generally produces smoother motion but may reduce tracking accuracy on sharp curves.

A smaller look-ahead distance can improve responsiveness but may cause more aggressive steering or oscillation.

### Linear Velocity

Linear velocity determines the robot's translational speed.

The appropriate velocity depends on the path geometry, controller behavior, localization accuracy, and the robot's physical limitations.

### Angular Velocity

Angular velocity determines the robot's rotational speed.

The controller should limit angular velocity to values that the robot can physically achieve.

### Wheel Geometry

The wheel positions, wheel dimensions, steering constraints, and kinematic parameters must accurately represent the physical robot.

Incorrect geometric parameters can cause discrepancies between the commanded motion and the robot's actual movement.

### Control Frequency

The control frequency determines how frequently the controller updates its commands.

The frequency should be appropriate for the robot's dynamics and the update rates of the localization and motor-control systems.

### Goal Tolerance

Goal tolerance determines when the robot considers itself to have reached its target or final waypoint.

The tolerance should account for localization accuracy and the requirements of the application.

**Note:** Exact parameter names and values depend on the controller implementation and its configuration files.

## ROS 2 Topics

The following table summarizes the principal interface identified for this project.

| Topic | Message Type | Purpose |
|---|---|---|
| `/plan` | `nav_msgs/Path` | Supplies the planned path to the path tracker |

Other topics depend on the implementation of the path tracker, localization system, and motor-control nodes.

### Inspect Available Topics

List all active ROS 2 topics:

```bash
ros2 topic list
```

Inspect the information for a specific topic:

```bash
ros2 topic info /plan
```

Display messages published on a topic:

```bash
ros2 topic echo /plan
```

Check the available nodes:

```bash
ros2 node list
```

These commands can help identify communication issues between the path tracker, localization system, and path publisher.

## Repository Structure

The repository follows a ROS 2 workspace layout.

```text
lbs_path_tracking_ws/
├── src/
│   └── <path_tracking_package>/
├── README.md
├── LICENSE
└── .gitignore
```

The exact contents of `src/` depend on the ROS 2 packages included in the repository.

The `<path_tracking_package>` entry is a placeholder for the actual package directory name.

The `build/`, `install/`, and `log/` directories are generated when the workspace is built. These directories generally should not be committed to Git.

## License

See the [LICENSE](LICENSE) file for license information.
