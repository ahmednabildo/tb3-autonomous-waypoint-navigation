# TurtleBot3 Autonomous Waypoint Navigation

A closed-loop autonomous waypoint navigation system for **TurtleBot3**, implemented using **MATLAB and ROS 2**.

The project implements proportional feedback control to guide a differential-drive mobile robot through a predefined sequence of waypoints. Each waypoint specifies a target position and orientation, both of which must be reached before the robot proceeds to the next waypoint.

The implementation also includes trajectory recording, velocity estimation, controller tuning experiments, and visualization of the robot's navigation performance.

---

## 1. Project Overview

The objective of this project is to develop a waypoint navigation controller for TurtleBot3 using real-time odometry feedback.

The system performs the following operations:

1. Loads predefined waypoints from an external text file.
2. Extracts the target positions and orientations.
3. Initializes ROS 2 communication using MATLAB.
4. Receives real-time robot odometry.
5. Calculates position and heading errors.
6. Generates proportional linear and angular velocity commands.
7. Navigates sequentially through the target waypoints.
8. Corrects the final orientation at each waypoint.
9. Records robot trajectories and velocity commands.
10. Visualizes and analyzes navigation performance.

The controller uses direct pose feedback without relying on a global path planner.

**Note:** SLAM, Nav2, and obstacle avoidance are not implemented in the current version.

---

## 2. Technologies and Tools

| Technology | Purpose |
|---|---|
| MATLAB | Controller development and data analysis |
| ROS 2 | Communication with the robot |
| MATLAB ROS Toolbox | ROS 2 publishers and subscribers |
| TurtleBot3 | Differential-drive mobile robot |
| Proportional Control | Linear and angular velocity regulation |
| Odometry | Robot pose feedback |
| Quaternion Conversion | Extracting heading from robot orientation |

---

## 3. Repository Structure

```text
tb3-autonomous-waypoint-navigation/
│
├── assignment2.m
├── waypoints.txt
├── eserc_7.pdf
│
├── Linear velocity.png
├── Angular velocity.png
├── measured_speed.png
│
├── Controller gain experiment plots/
│
└── .gitattributes
```

The controller gain experiment plots are stored under gain-specific names in the repository.

### Main Files

**`assignment2.m`**

Main MATLAB implementation containing:

- Waypoint file parsing.
- ROS 2 node initialization.
- Odometry subscription.
- Velocity command publishing.
- Proportional navigation control.
- Sequential waypoint execution.
- Trajectory recording.
- Velocity estimation.
- Performance visualization.

**`waypoints.txt`**

Contains predefined waypoint sequences for both TurtleSim and TurtleBot3.

**`eserc_7.pdf`**

Original assignment documentation.

**Result images**

Saved plots of the robot trajectory, position, orientation, and velocity responses.

---

## 4. Robot Mathematical Model

### 4.1 Robot Configuration

The configuration of the differential-drive mobile robot is represented by:

```math
\mathbf{q} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
```

where:

- `x`: Robot position along the global X-axis.
- `y`: Robot position along the global Y-axis.
- `θ`: Robot heading angle.

The desired waypoint is defined as:

```math
\mathbf{q}_g =
\begin{bmatrix}
x_g \\
y_g \\
\theta_g
\end{bmatrix}
```

where:

- `x_g`: Desired X position.
- `y_g`: Desired Y position.
- `θ_g`: Desired final orientation.

### 4.2 Differential-Drive Kinematic Model

The planar motion of the mobile robot can be described using:

```math
\dot{x} = v\cos(\theta)
```

```math
\dot{y} = v\sin(\theta)
```

```math
\dot{\theta} = \omega
```

where:

- `v`: Forward linear velocity.
- `ω`: Angular velocity.
- `θ`: Heading angle.

The model can be expressed in matrix form:

```math
\dot{\mathbf{q}} =
\begin{bmatrix}
\cos(\theta) & 0 \\
\sin(\theta) & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
```

The controller generates the linear and angular velocity commands required to navigate toward each target pose.

---

## 5. Waypoint Navigation Controller

The navigation controller uses proportional feedback for both position and orientation regulation.

The navigation strategy consists of two main stages:

1. Move toward the target position.
2. Rotate to the requested final orientation.

### 5.1 Position Error

The distance between the robot's current position and the target waypoint is calculated as:

```math
e_p =
\sqrt{
(x_g-x)^2+(y_g-y)^2
}
```

This quantity represents the Euclidean distance to the target.

### 5.2 Desired Heading

The desired heading toward the target is:

```math
\theta_d =
\mathrm{atan2}(y_g-y,\ x_g-x)
```

The heading error is:

```math
e_\theta =
\mathrm{atan2}
\left(
\sin(\theta_d-\theta),
\cos(\theta_d-\theta)
\right)
```

This formulation wraps the heading error to the interval:

```math
e_\theta \in [-\pi,\pi]
```

Consequently, the robot rotates through the shortest angular displacement toward the desired direction.

### 5.3 Linear Velocity Controller

The commanded linear velocity is calculated using proportional feedback:

```math
v_{\mathrm{cmd}} =
K_{p,v}e_p
```

The maximum linear velocity is limited by:

```math
v_{\mathrm{cmd}} =
\min(K_{p,v}e_p,\ v_{\max})
```

where:

- `Kp,v`: Linear proportional gain.
- `e_p`: Position error.
- `v_max`: Maximum allowable linear velocity.

The controller also considers the heading error.

If the heading error exceeds the driving tolerance, the forward velocity is set to zero.

```math
v_{\mathrm{cmd}} =
\begin{cases}
0,
& |e_\theta| > \varepsilon_d \\
\min(K_{p,v}e_p,v_{\max}),
& |e_\theta| \leq \varepsilon_d
\end{cases}
```

This expression applies during the position-navigation stage, while the position error exceeds its tolerance.

Here, `ε_d` represents the allowable heading error before forward motion.

This approach prevents the robot from moving forward while significantly misaligned with the target.

### 5.4 Angular Velocity Controller

The commanded angular velocity is calculated as:

```math
\omega_{\mathrm{cmd}} =
K_{p,\theta}e_\theta
```

where:

- `Kp,θ`: Angular proportional gain.
- `e_θ`: Heading error.

To limit the commanded rotational speed, angular velocity saturation is applied:

```math
\omega_{\mathrm{cmd}} =
\max
\left(
-\omega_{\max},
\min
\left(
K_{p,\theta}e_\theta,
\omega_{\max}
\right)
\right)
```

where `ω_max` is the maximum allowable angular velocity.

The controller supports both clockwise and counterclockwise rotations.

### 5.5 Final Orientation Control

When the robot reaches the target position, the controller stops its forward motion:

```math
v_{\mathrm{cmd}} = 0
```

The heading error is then calculated relative to the requested final waypoint orientation:

```math
e_{\theta,g} =
\mathrm{atan2}
\left(
\sin(\theta_g-\theta),
\cos(\theta_g-\theta)
\right)
```

The robot rotates in place until:

```math
|e_{\theta,g}| \leq \varepsilon_\theta
```

where `ε_θ` represents the final heading tolerance.

### 5.6 Waypoint Completion Conditions

A waypoint is considered reached when both position and orientation requirements are satisfied.

**Position condition:**

```math
e_p \leq \varepsilon_p
```

**Orientation condition:**

```math
|e_{\theta,g}| \leq \varepsilon_\theta
```

where:

- `ε_p`: Position tolerance.
- `ε_θ`: Final heading tolerance.

Once both conditions are satisfied, the controller proceeds to the next waypoint.

---

## 6. Controller Parameters

The current MATLAB implementation uses the following controller parameters:

| Parameter | Value | Description |
|---|---|---|
| `kpv` | 0.1 | Linear proportional gain |
| `kptheta` | 0.3 | Angular proportional gain |
| `vmax` | 0.2 m/s | Maximum linear velocity |
| `omegaMax` | 0.4 rad/s | Maximum angular velocity |
| `positionTolerance` | 0.05 m | Maximum position error |
| `headingTolerance` | 0.005 rad | Final orientation tolerance |
| `driveHeadingTolerance` | 0.1 rad | Heading tolerance before forward motion |

The parameters are defined in the MATLAB controller function:

```matlab
kpv = 0.1;
kptheta = 0.3;

vmax = 0.2;
omegaMax = 0.4;

positionTolerance = 0.05;
headingTolerance = 0.005;
driveHeadingTolerance = 0.1;
```

Additional controller tuning experiments were performed using different gains, including:

```matlab
kpv = 0.5;
kptheta = 1.0;
```

Changing the proportional gains affects the commanded velocities and the robot's navigation response.

---

## 7. ROS 2 Communication

The MATLAB controller communicates with TurtleBot3 through ROS 2 publishers and subscribers.

### 7.1 ROS 2 Node

A ROS 2 node is created using:

```matlab
node = ros2node('/waypoint_controller');
```

The node handles the communication between MATLAB and the robot.

### 7.2 Odometry Subscriber

The robot state is obtained from the `/odom` topic.

| Topic | Message Type | Purpose |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Robot pose feedback |

Subscriber initialization:

```matlab
odomSub = ros2subscriber( ...
    node, '/odom', 'nav_msgs/Odometry');
```

The controller extracts the robot position:

```matlab
x = odomMsg.pose.pose.position.x;
y = odomMsg.pose.pose.position.y;
```

The orientation is provided as a quaternion.

### 7.3 Quaternion-to-Euler Conversion

ROS 2 represents the robot orientation using a quaternion:

```math
\mathbf{q}_{\mathrm{quat}} =
\begin{bmatrix}
q_w \\
q_x \\
q_y \\
q_z
\end{bmatrix}
```

The quaternion is converted into Euler angles using MATLAB:

```matlab
q = odomMsg.pose.pose.orientation;

eulerAngles = quat2eul( ...
    [q.w, q.x, q.y, q.z], 'ZYX');

theta = eulerAngles(1);
```

The first Euler angle corresponds to the yaw angle, representing the robot heading.

### 7.4 Velocity Command Publisher

The controller sends velocity commands through the `/cmd_vel` topic.

| Topic | Message Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands |

Publisher initialization:

```matlab
cmdPub = ros2publisher( ...
    node, '/cmd_vel', ...
    'geometry_msgs/TwistStamped');
```

The computed commands are published as:

```matlab
cmdMsg = ros2message('geometry_msgs/TwistStamped');

cmdMsg.header.stamp = ros2time(node, 'now');

cmdMsg.twist.linear.x = vlinear;
cmdMsg.twist.angular.z = omega;

send(cmdPub, cmdMsg);
```

**Important:** The current implementation uses `geometry_msgs/TwistStamped`. Some ROS 2 robot configurations require `geometry_msgs/Twist` instead.

The expected message type must be verified before execution.

### 7.5 Odometry Timeout Handling

The controller waits for a fresh odometry message with a one-second timeout.

If no odometry message is received:

1. A zero-velocity command is published.
2. An odometry timeout error is raised.
3. The active controller execution terminates.

This prevents normal navigation from continuing without updated odometry feedback.

---

## 8. Waypoint File

The target waypoints are defined in `waypoints.txt`.

The file contains separate sequences for TurtleSim and TurtleBot3.

Each waypoint contains:

```text
index    x;y;theta
```

Example TurtleBot3 waypoints:

```text
turtlebot3 = x    y   theta

init    0;0;0
1       5;0;pi/2
2       5;5;5/4pi
3       -5;-5;pi/2
4       -5;5;0
5       0;0;0
6       3;3;3/4pi
7       -3;0;3/2pi
8       0;-3;pi/4
9       3;0;pi/2
10      0;0;3/2pi
```

The waypoint parser supports:

- Initial robot poses.
- Numbered target waypoints.
- Expressions containing `pi`.
- Decimal comma notation.
- Comments.
- Separate waypoint lists for TurtleSim and TurtleBot3.

The initial conditions and target waypoints are stored separately:

```matlab
turtlesim_initial
turtlesim_waypoints

turtlebot3_initial
turtlebot3_waypoints
```

Although both waypoint sequences are loaded and visualized, the implemented controller executes the TurtleBot3 sequence.

The initial TurtleBot3 pose specified in the file is used for reference plotting. It does not automatically reset the robot's physical or simulated pose.

---

## 9. Navigation Algorithm

### 9.1 Initialization

The program begins by:

1. Loading the waypoint file.
2. Extracting the TurtleBot3 target poses.
3. Plotting the reference waypoint sequence.
4. Creating the ROS 2 node.
5. Initializing the odometry subscriber.
6. Initializing the velocity command publisher.

### 9.2 Sequential Waypoint Execution

For each waypoint, the controller:

1. Receives the latest odometry message.
2. Extracts the current robot position and orientation.
3. Calculates the position error.
4. Determines the desired heading.
5. Calculates the heading error.
6. Computes linear and angular velocity commands.
7. Applies the configured velocity limits.
8. Publishes the commands through ROS 2.
9. Repeats until the target position is reached.
10. Rotates to the desired final orientation.
11. Advances to the next waypoint.

### 9.3 Navigation Pseudocode

```text
LOAD waypoint file

EXTRACT TurtleBot3 waypoints

INITIALIZE ROS 2 node

CREATE odometry subscriber

CREATE velocity command publisher

FOR each target waypoint:

    reached = false

    WHILE reached == false:

        RECEIVE current odometry

        EXTRACT x, y, theta

        CALCULATE position error

        IF position error > position tolerance:

            CALCULATE desired heading

            CALCULATE heading error

            CALCULATE proportional linear velocity

            IF heading error > drive heading tolerance:

                SET linear velocity = 0

        ELSE:

            SET linear velocity = 0

            CALCULATE final heading error

            IF final heading error <= heading tolerance:

                reached = true

        CALCULATE proportional angular velocity

        APPLY angular velocity saturation

        IF reached:

            SET angular velocity = 0

        PUBLISH velocity commands

        RECORD robot pose and commands

END FOR

PLOT measured trajectory

PLOT position and orientation responses

ESTIMATE measured velocities

COMPARE commanded and estimated velocities
```

The controller function completes one target waypoint before returning to the main script.

---

## 10. Installation and Execution

### 10.1 Prerequisites

The project requires:

- MATLAB.
- MATLAB ROS Toolbox.
- A compatible ROS 2 environment.
- TurtleBot3 simulation or physical robot.
- An available `/odom` topic.
- A compatible `/cmd_vel` interface.

The repository does not include a complete TurtleBot3 simulation package or ROS 2 launch configuration.

### 10.2 Clone the Repository

Clone the project:

```bash
git clone https://github.com/ahmednabildo/tb3-autonomous-waypoint-navigation.git
```

Navigate to the project directory:

```bash
cd tb3-autonomous-waypoint-navigation
```

### 10.3 Start the TurtleBot3 Environment

Start the appropriate TurtleBot3 simulation or physical robot environment.

Verify the ROS 2 topics:

```bash
ros2 topic list
```

Check the odometry interface:

```bash
ros2 topic info /odom
```

Check the velocity command interface:

```bash
ros2 topic info /cmd_vel
```

The message types must match the types used in the MATLAB implementation.

### 10.4 Open MATLAB

Set the current working directory to the repository:

```matlab
cd('/path/to/tb3-autonomous-waypoint-navigation')
```

Make sure the following files are available:

```text
assignment2.m
waypoints.txt
```

### 10.5 Run the Controller

Execute the MATLAB script:

```matlab
run('assignment2.m')
```

The script will:

1. Parse the waypoint file.
2. Plot the target waypoint paths.
3. Initialize ROS 2 communication.
4. Start waypoint navigation.
5. Record the measured robot trajectory.
6. Store the velocity commands.
7. Generate plots after navigation finishes.

**Safety note:** The predefined waypoints may require a large obstacle-free workspace. Adjust the waypoint coordinates before running the controller on a physical robot or in a smaller simulation environment.

---

## 11. Results and Performance Analysis

The MATLAB implementation records robot states and velocity commands throughout navigation.

These data are used to evaluate the navigation response.

### 11.1 Reference Waypoint Visualization

The waypoint sequences are visualized in the XY plane.

The plots display:

- Initial robot position.
- Target waypoint locations.
- Reference waypoint connections.

Separate plots are generated for TurtleSim and TurtleBot3.

### 11.2 Measured Robot Trajectory

The robot's measured planar position is represented by:

```math
\mathbf{p}(t) =
\begin{bmatrix}
x(t) \\
y(t)
\end{bmatrix}
```

The trajectory is reconstructed from the received odometry measurements.

The measured trajectory is plotted against the target waypoint locations.

This visualization allows qualitative inspection of the robot's navigation behavior.

### 11.3 Position and Orientation Responses

The controller records the robot pose over time:

```math
\mathbf{q}(t) =
\begin{bmatrix}
x(t) \\
y(t) \\
\theta(t)
\end{bmatrix}
```

The MATLAB script plots:

- X-position versus time.
- Y-position versus time.
- Heading versus time.
- Commanded linear velocity versus time.
- Commanded angular velocity versus time.

These plots provide insight into how the robot moves and rotates during navigation.

### 11.4 Linear Velocity Estimation

The robot's linear speed is estimated numerically from its recorded position.

```math
\hat{v}(t) =
\sqrt{
\left(\frac{dx}{dt}\right)^2
+
\left(\frac{dy}{dt}\right)^2
}
```

The derivatives are approximated using MATLAB's `gradient` function.

```matlab
vxMeasured = gradient(xMeasured, t);

vyMeasured = gradient(yMeasured, t);

vMeasured = hypot(vxMeasured, vyMeasured);
```

The estimated linear speed is compared with the commanded forward velocity.

![Linear Velocity Comparison](Linear%20velocity.png)

This comparison helps analyze the relationship between controller commands and the robot's observed movement.

The estimated speed is a nonnegative magnitude and does not distinguish forward from backward motion.

### 11.5 Angular Velocity Estimation

Angular velocity is estimated by differentiating the robot heading.

```math
\hat{\omega}(t) =
\frac{d}{dt}
\left(
\mathrm{unwrap}(\theta(t))
\right)
```

The `unwrap` operation prevents artificial discontinuities when the heading crosses the angle wrapping boundary.

MATLAB implementation:

```matlab
thetaMeasured = unwrap(trajectory(sampleIdx, 3));

omegaMeasured = gradient(thetaMeasured, t);
```

The estimated angular velocity is compared with the commanded angular velocity.

![Angular Velocity Comparison](Angular%20velocity.png)

These measurements are derived from odometry position and orientation rather than direct velocity measurements.

Numerical differentiation can amplify measurement noise and introduce inaccuracies.

### 11.6 Controller Gain Experiments

The repository includes additional plots corresponding to different controller gains.

One investigated configuration is:

```matlab
kpv = 0.5;
kptheta = 1.0;
```

The saved figures include:

- Position responses.
- Heading response.
- Linear velocity response.
- Angular velocity response.
- Robot trajectory.

These plots support comparisons of how controller gains influence the robot's motion.

---

## 12. Limitations

The current implementation has the following limitations:

- Waypoints are predefined.
- Global path planning is not implemented.
- Obstacle avoidance is not implemented.
- Localization depends on odometry feedback.
- No SLAM system is included.
- The waypoint file does not automatically initialize the robot pose.
- The controller does not implement a general recovery strategy for unreachable targets.
- No automatic telemetry export is implemented.
- The ROS 2 command interface depends on the robot configuration.
- Numerical velocity estimates may be affected by odometry noise.

The project focuses on waypoint tracking and feedback control rather than a complete autonomous navigation stack.

---

## 13. Future Improvements

Possible extensions include:

- Integration with path-planning algorithms.
- Obstacle detection and avoidance.
- Sensor fusion for improved state estimation.
- Integration with SLAM frameworks.
- Automated controller gain tuning.
- Quantitative position and heading error analysis.
- Automatic experimental data export.
- Improved navigation failure handling.
- Comparison with nonlinear control strategies.
- Integration with ROS 2 Nav2.
- Visualization of robot tracking errors in real time.

---

## 14. Project Context

This project was developed as part of a mobile robotics exercise involving waypoint navigation, proportional feedback control, and ROS 2 integration.

The implementation demonstrates experience with:

- Mobile robot kinematics.
- Closed-loop feedback control.
- Autonomous waypoint navigation.
- MATLAB robotics programming.
- ROS 2 communication.
- Odometry-based pose estimation.
- Quaternion transformations.
- Controller parameter tuning.
- Robot trajectory analysis.
- Numerical velocity estimation.

The original assignment documentation is included in [`eserc_7.pdf`](eserc_7.pdf).

---

## Author

**Ahmed Nabil Donia**

MSc Mechanical Engineering — Mechatronics Track  
Politecnico di Milano

[GitHub Profile](https://github.com/ahmednabildo)
