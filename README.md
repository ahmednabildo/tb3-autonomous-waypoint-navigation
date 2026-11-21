# TurtleBot3 Autonomous Waypoint Navigation

Closed-loop waypoint tracking for **TurtleBot3** using **MATLAB** and **ROS 2**. A proportional controller uses live odometry to guide the robot through a predefined sequence of target poses, aligning with each waypoint and achieving its requested final orientation before proceeding to the next.

The repository also includes trajectory visualizations and comparisons of commanded velocities with velocities estimated from the recorded pose.

## Overview

The project demonstrates a feedback-based approach to planar mobile-robot navigation. The MATLAB script:

1. Parses a text file containing initial poses and waypoints for TurtleSim and TurtleBot3.
2. Plots both sets of planned waypoint locations.
3. Creates a ROS 2 node, subscribes to TurtleBot3 odometry, and publishes motion commands.
4. Drives the TurtleBot3 through its target poses sequentially.
5. Logs the measured pose and associated velocity commands.
6. Plots the measured trajectory and evaluates commanded versus numerically estimated velocities.

**Scope:** This is a reactive pose-feedback waypoint controller. It does **not** implement SLAM, global path planning, Nav2, or obstacle avoidance. The current script executes the **TurtleBot3** waypoint list; the TurtleSim list is parsed and plotted but is not sent to a TurtleSim controller.

## Technologies

- MATLAB
- ROS 2
- MATLAB ROS Toolbox (ROS 2 node, publisher, subscriber, and message interfaces)
- MATLAB quaternion/Euler-angle conversion (`quat2eul`)
- TurtleBot3 simulator or robot exposing the required ROS 2 interfaces

## Repository Contents

```text
tb3-autonomous-waypoint-navigation/
├── assignment2.m           # Main MATLAB script and feedback controller
├── waypoints.txt           # Initial poses and waypoint lists
├── eserc_7.pdf             # Assignment document
├── Linear velocity.png     # Commanded vs. estimated linear velocity
├── Angular velocity.png    # Commanded vs. estimated angular velocity
├── measured_speed.png      # Additional saved result plot
├── ...                     # Additional pose, trajectory, and gain-specific plots
└── .gitattributes
```

The source code is contained in `assignment2.m`, including the local `turtlebot_controller` function. Other PNG files in the repository illustrate trajectories and controller responses for particular gain settings.

## Control Method

Each target waypoint is a desired planar pose

\[
\mathbf{q}_g = \begin{bmatrix}x_g & y_g & \theta_g\end{bmatrix}^{\mathsf T}.
\]

The robot pose \((x,y,\theta)\) is obtained from `/odom`; the quaternion orientation is converted to yaw using the ZYX Euler-angle convention.

### 1. Position and heading errors

The distance to the active waypoint is

\[
e_p = \sqrt{(x_g-x)^2+(y_g-y)^2}.
\]

While traveling toward the waypoint, the desired direction and wrapped angular error are

\[
\theta_d = \operatorname{atan2}(y_g-y,\,x_g-x),
\]

\[
e_\theta = \operatorname{atan2}\!\left(\sin(\theta_d-\theta),\cos(\theta_d-\theta)\right).
\]

After reaching the position tolerance, the controller instead uses the requested final orientation:

\[
e_\theta = \operatorname{atan2}\!\left(\sin(\theta_g-\theta),\cos(\theta_g-\theta)\right).
\]

### 2. Proportional velocity commands

The commanded forward velocity is proportional to the distance error, subject to saturation:

\[
v = \min(K_{p,v}e_p, v_{\max}).
\]

If the robot is facing more than the drive-heading tolerance away from the waypoint, forward velocity is set to zero while it turns. Once it is within the position tolerance, forward motion is also stopped and the robot rotates to its final target heading.

The commanded angular velocity is

\[
\omega = \operatorname{clip}(K_{p,\theta} e_\theta,-\omega_{\max},\omega_{\max}).
\]

A waypoint is completed when both position and final-heading requirements have been satisfied; the controller then proceeds to the next target.

### Parameters in `assignment2.m`

| Parameter | Value | Meaning |
|---|---:|---|
| `kpv` | `0.1` | Linear proportional gain |
| `kptheta` | `0.3` | Angular proportional gain |
| `vmax` | `0.2 m/s` | Maximum forward speed |
| `omegaMax` | `0.4 rad/s` | Maximum absolute angular speed |
| `positionTolerance` | `0.05 m` | Accepted distance to target |
| `headingTolerance` | `0.005 rad` | Accepted final-heading error |
| `driveHeadingTolerance` | `0.1 rad` | Maximum heading error before forward motion is paused |

These are the **current values in the source code**. The repository also contains plots labeled with other gain combinations, illustrating controller-tuning experiments.

## ROS 2 Interfaces

| Interface | ROS 2 message type | Role |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Subscribed: measured robot position and orientation |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Published: commanded forward and angular velocity |

The MATLAB node is named `/waypoint_controller`. It timestamps outgoing velocity commands with the ROS 2 node's current time. The controller reads a new odometry message for every control iteration. If no odometry message arrives within **one second**, it publishes a zero-velocity command and raises an error.

**Compatibility note:** This script publishes `TwistStamped`, not `Twist`. Confirm that the target robot or simulation accepts `geometry_msgs/TwistStamped` on `/cmd_vel`, or adapt the message type to your specific ROS 2 configuration.

## Waypoint File

`waypoints.txt` contains two separately labeled groups: `turtlesim` and `turtlebot3`. Each entry contains

```text
index    x;y;theta
```

For example, the TurtleBot3 section begins:

```text
turtlebot3 = x    y   theta
    init  0;0;0
    1     5;0;pi/2
    2     5;5;5/4pi
    3     -5;-5;pi/2
```

Coordinates are interpreted as metres and heading as radians for TurtleBot3. The parser supports expressions involving `pi` and converts decimal commas where present. The `init` entries are separated from the target waypoints; the actual TurtleBot3 control loop uses its live odometry pose rather than commanding the `init` entry.

**Important:** Initial poses in the file are reference values, not commands that reset the simulation or teleport the robot.

## Running the Project

### Prerequisites

- A running ROS 2 environment with a TurtleBot3 simulator or physical robot.
- MATLAB with ROS Toolbox and the functions required by `assignment2.m`.
- An active `/odom` topic publishing `nav_msgs/Odometry` messages.
- A compatible `/cmd_vel` consumer accepting `geometry_msgs/TwistStamped`.
- ROS 2 discovery/network settings configured so MATLAB can communicate with the robot.

The repository does not include a TurtleBot3 launch file or a ROS 2 package; launch the robot environment separately using the instructions appropriate to your ROS 2 distribution and TurtleBot3 setup.

### 1. Clone

```bash
git clone https://github.com/ahmednabildo/tb3-autonomous-waypoint-navigation.git
cd tb3-autonomous-waypoint-navigation
```

### 2. Start the robot environment

Start your TurtleBot3 simulator/robot and verify that `/odom` is being published and `/cmd_vel` accepts the expected message type. For example, in a sourced ROS 2 terminal:

```bash
ros2 topic list
ros2 topic info /odom
ros2 topic info /cmd_vel
```

### 3. Run the MATLAB script

Open MATLAB in the repository root (or set its current working directory to the repository root). The code reads the waypoints using `fullfile(pwd, 'waypoints.txt')`, so **`waypoints.txt` must be in MATLAB's current folder**.

```matlab
cd('/path/to/tb3-autonomous-waypoint-navigation')
run('assignment2.m')
```

The script first shows waypoint plots, then connects to ROS 2 and begins sequentially commanding the TurtleBot3 waypoints. **Only run it when the operating area is appropriate for the configured waypoints and the robot can be stopped safely.**

## Outputs and Analysis

After completing the waypoint sequence, the script generates:

- **Waypoint plans** for TurtleSim and TurtleBot3.
- **Measured TurtleBot3 trajectory** in the XY plane, overlaid with the target waypoints.
- **Pose histories** (`x`, `y`, and heading) and commanded linear/angular velocity versus odometry time.
- **Commanded vs. estimated linear speed**, where measured speed is estimated from finite differences of the recorded XY position:

  \[
  \hat v = \sqrt{\dot x^2+\dot y^2}.
  \]

- **Commanded vs. estimated angular speed**, estimated by differentiating the unwrapped measured heading:

  \[
  \hat\omega = \frac{d}{dt}\operatorname{unwrap}(\theta).
  \]

These velocity estimates are calculated numerically using MATLAB's `gradient` function. They are not direct wheel-encoder velocity measurements, and numerical differentiation can be sensitive to sampling and odometry noise.

### Example figures

**Commanded versus estimated linear velocity**

![Commanded versus estimated linear velocity](Linear%20velocity.png)

**Commanded versus estimated angular velocity**

![Commanded versus estimated angular velocity](Angular%20velocity.png)

The repository also contains saved plots for a gain combination labeled `kpv = 0.5; kptheta = 1.0;`. These are historical result artifacts; the presently committed controller defaults are `kpv = 0.1` and `kptheta = 0.3`.

## Current Limitations

- The controller has no collision checking or obstacle avoidance. Waypoints must be feasible in the selected environment.
- The waypoint sequence is fixed in `waypoints.txt` and executed in order.
- The MATLAB script does not launch or reset the robot simulation.
- Timeout handling covers missing odometry messages; the source does not implement a broader autonomous recovery strategy.
- Logged plots are generated after the entire target sequence is completed; individual runs are not automatically exported to data files by the script.
- ROS 2 velocity message compatibility depends on the robot configuration.

## Possible Extensions

Potential next steps include obstacle-aware planning, live visualization of waypoint errors, configurable gains and safety limits, automatic export of telemetry, and comparative evaluation of different controllers.

## Project Context

A waypoint-navigation and feedback-control exercise implemented in MATLAB with a ROS 2 TurtleBot3 interface. See [`eserc_7.pdf`](eserc_7.pdf) for the assignment material included in the repository.
