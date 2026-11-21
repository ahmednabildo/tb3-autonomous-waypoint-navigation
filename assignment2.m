clc;
clear;
close all;

%% 1. Load waypoint file
filePath = fullfile(pwd, 'waypoints.txt');
lines = readlines(filePath);

%% 2. Initialize the two matrices
turtlesim = [];
turtlebot3 = [];

robot = "";

%% 3. Extract the waypoints
for i = 1:length(lines)

    line = strtrim(lines(i));

    % Identify the current robot
    if startsWith(line, "turtlesim")
        robot = "turtlesim";
        continue;

    elseif startsWith(line, "turtlebot3")
        robot = "turtlebot3";
        continue;
    end

    % Remove comments
    line = strtrim(extractBefore(line + "#", "#"));

    % Extract x, y and theta
    parts = regexp(char(line), ...
        '^(?:init|\d+)\s+([^;]+);([^;]+);([^;]+)$', ...
        'tokens', 'once');

    if isempty(parts)
        continue;
    end

    % Convert coordinates to numbers
    x = str2double(strrep(strtrim(parts{1}), ',', '.'));
    y = str2double(strrep(strtrim(parts{2}), ',', '.'));

    % Convert theta expressions containing pi
    thetaExpr = strtrim(parts{3});
    thetaExpr = regexprep(thetaExpr, '(\d)pi', '$1*pi');
    theta = str2num(thetaExpr);

    % Store the waypoint
    waypoint = [x, y, theta];

    if robot == "turtlesim"
        turtlesim = [turtlesim; waypoint];

    elseif robot == "turtlebot3"
        turtlebot3 = [turtlebot3; waypoint];
    end

end

%% 4. Separate initial conditions from target waypoints

turtlesim_initial = turtlesim(1, :);
turtlesim_waypoints = turtlesim(2:end, :);

turtlebot3_initial = turtlebot3(1, :);
turtlebot3_waypoints = turtlebot3(2:end, :);

%% 5. x,y,theat

x_t_b = turtlebot3(: , 1);
y_t_b = turtlebot3(: , 2);
theta_t_b = turtlebot3(: , 3);

x_t_s = turtlesim(: , 1);
y_t_s = turtlesim(: , 2);
theta_t_s = turtlesim(: , 3);


%% 6. Plot the initial positions and waypoint paths
figure('Name', 'TurtleSim and TurtleBot3 waypoints');

subplot(1, 2, 1);
plot(x_t_s, y_t_s, 'b-o', 'LineWidth', 1.5);
hold on;
plot(turtlesim_initial(1), turtlesim_initial(2), 'rp', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'r');
hold off;
axis equal;
grid on;
xlabel('x');
ylabel('y');
title('TurtleSim');
legend('Waypoint path', 'Initial position', 'Location', 'best');

subplot(1, 2, 2);
plot(x_t_b, y_t_b, 'b-o', 'LineWidth', 1.5);
hold on;
plot(turtlebot3_initial(1), turtlebot3_initial(2), 'rp', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'r');
hold off;
axis equal;
grid on;
xlabel('x');
ylabel('y');
title('TurtleBot3');
legend('Waypoint path', 'Initial position', 'Location', 'best');
%% 7. Create the ROS 2 node, odometry subscriber and command publisher
node = ros2node('/waypoint_controller');

odomSub = ros2subscriber(node, '/odom', 'nav_msgs/Odometry');

cmdPub = ros2publisher(node, '/cmd_vel','geometry_msgs/TwistStamped');


%% Main: each controller call completes one target waypoint
trajectory = zeros(0, 3); % Columns: x, y, theta (radians)
commands = zeros(0, 3);   % Columns: odometry time (s), vcmd (m/s), omega (rad/s)
for i = 1:size(turtlebot3_waypoints, 1)
    xg = turtlebot3_waypoints(i, 1);
    yg = turtlebot3_waypoints(i, 2);
    thetag = turtlebot3_waypoints(i, 3);

    [vlinear, omega, reached, waypointTrajectory, waypointCommands] = ...
        turtlebot_controller(xg, yg, thetag, odomSub, cmdPub, node);
    trajectory = [trajectory; waypointTrajectory];
    commands = [commands; waypointCommands];
end

%% Plot the measured trajectory
figure('Name', 'TurtleBot3 measured trajectory');
plot(trajectory(:, 1), trajectory(:, 2), 'b-', 'LineWidth', 1.5);
hold on;
plot(turtlebot3_waypoints(:, 1), turtlebot3_waypoints(:, 2), ...
    'rx', 'MarkerSize', 8, 'LineWidth', 1.5);
hold off;
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('TurtleBot3 measured trajectory');
legend('Measured trajectory', 'Target waypoints', 'Location', 'best');

%% Plot position, heading and velocity commands against simulation time
time = commands(:, 1) - commands(1, 1); % Seconds since the first odometry sample
figure('Name', 'TurtleBot3 pose and velocity commands');

subplot(3, 2, 1);
plot(time, trajectory(:, 1), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('x versus time');

subplot(3, 2, 2);
plot(time, trajectory(:, 2), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('y (m)');
title('y versus time');

subplot(3, 2, [3, 4]);
plot(time, trajectory(:, 3), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Heading versus time');

subplot(3, 2, 5);
stairs(time, commands(:, 2), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('v_{cmd} (m/s)');
title('Commanded linear velocity');

subplot(3, 2, 6);
stairs(time, commands(:, 3), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('\omega_{cmd} (rad/s)');
title('Commanded angular velocity');

%% Estimate measured velocities from the logged pose
[t, sampleIdx] = unique(time(:), 'stable');
assert(numel(t) >= 2 && all(diff(t) > 0), ...
    'Velocity estimation requires at least two increasing timestamps.');
xMeasured = trajectory(sampleIdx, 1);
yMeasured = trajectory(sampleIdx, 2);
thetaMeasured = unwrap(trajectory(sampleIdx, 3));

vxMeasured = gradient(xMeasured, t);
vyMeasured = gradient(yMeasured, t);
vMeasured = hypot(vxMeasured, vyMeasured);
omegaMeasured = gradient(thetaMeasured, t);
tCmd = time;

%% Compare commanded and measured linear velocity
figure('Name', 'Linear velocity', 'WindowState', 'maximized');
plot(t, vMeasured, 'b-', 'LineWidth', 3);
hold on;
stairs(tCmd, commands(:, 2), 'r--', 'LineWidth', 1.8);
hold off;
grid on;
xlabel('Time (s)');
ylabel('v (m/s)');
title('Linear velocity: commanded versus measured');
legend('Measured estimate', 'Commanded', 'Location', 'best');
set(gca, 'FontSize', 14);
zoom on;

%% Compare commanded and measured angular velocity
figure('Name', 'Angular velocity', 'WindowState', 'maximized');
plot(t, omegaMeasured, 'b-', 'LineWidth', 3);
hold on;
stairs(tCmd, commands(:, 3), 'r--', 'LineWidth', 1.8);
hold off;
grid on;
xlabel('Time (s)');
ylabel('\omega (rad/s)');
title('Angular velocity: commanded versus measured');
legend('Measured estimate', 'Commanded', 'Location', 'best');
set(gca, 'FontSize', 14);
zoom on;


%% 8. Controller function
function [vlinear, omega, reached, waypointTrajectory, waypointCommands] = ...
    turtlebot_controller(xg, yg, thetag, odomSub, cmdPub, node)
%TURTLEBOT_CONTROLLER Drive to one waypoint using ROS 2 odometry feedback.
%   xg, yg, thetag: target waypoint pose (metres, radians).
%   odomSub: ROS 2 subscriber for /odom, nav_msgs/Odometry.
%   cmdPub: ROS 2 publisher for /cmd_vel, geometry_msgs/TwistStamped.
%   node: ROS 2 node used to timestamp the command.
%   Current x, y and heading are read from /odom inside this function.
%   Loops internally and returns after position and final heading are reached.
%   waypointTrajectory contains one measured [x, y, theta] row per iteration.
%   waypointCommands contains matching [odometry time, vlinear, omega] rows.
%
%   The node, subscriber and publisher are created in assignment2.m,
%   section 7, before this function. Call once for each waypoint:
%     [vlinear, omega, reached, waypointTrajectory, waypointCommands] = ...
%         turtlebot_controller(xg, yg, thetag, odomSub, cmdPub, node);

% Proportional gains and control constraints.
kpv = 0.1;
kptheta = 0.3;
vmax = 0.2;                  % m/s
omegaMax = 0.4;              % rad/s
positionTolerance = 0.05;    % m
headingTolerance = 0.005;     % rad, final heading tolerance
driveHeadingTolerance = 0.1; % rad, align before driving

waypointTrajectory = zeros(0, 3);
waypointCommands = zeros(0, 3);
reached = false;
while ~reached
    % Read a fresh odometry measurement (one-second timeout).
    [odomMsg, received] = receive(odomSub, 1);
    if ~received
        stopMsg = ros2message('geometry_msgs/TwistStamped');
        stopMsg.header.stamp = ros2time(node, 'now');
        send(cmdPub, stopMsg);
        error('turtlebot_controller:OdometryTimeout', ...
            'No fresh /odom message received within one second.');
    end

    x = odomMsg.pose.pose.position.x;
    y = odomMsg.pose.pose.position.y;

    % ROS stores orientation as x, y, z, w; quat2eul expects [w x y z].
    q = odomMsg.pose.pose.orientation;
    eulerAngles = quat2eul([q.w, q.x, q.y, q.z], 'ZYX');
    theta = eulerAngles(1); % ZYX returns [yaw pitch roll], in radians.

    waypointTrajectory(end + 1, :) = [x, y, theta];

    errorv = hypot(xg - x, yg - y);
    if errorv > positionTolerance
        % First face the waypoint, then drive towards it.
        desiredTheta = atan2(yg - y, xg - x);
        errortheta = atan2(sin(desiredTheta - theta), cos(desiredTheta - theta));

        vlinear = min(kpv * errorv, vmax);
        if abs(errortheta) > driveHeadingTolerance
            vlinear = 0;
        end
    else
        % Position reached: rotate in place to the requested final heading.
        vlinear = 0;
        errortheta = atan2(sin(thetag - theta), cos(thetag - theta));
        reached = abs(errortheta) <= headingTolerance;
    end

    % Saturate both clockwise and counterclockwise angular commands.
    omega = max(-omegaMax, min(kptheta * errortheta, omegaMax));
    if reached
        omega = 0;
    end

    % Fresh messages keep all unused linear/angular components at zero.
    cmdMsg = ros2message('geometry_msgs/TwistStamped');
    cmdMsg.header.stamp = ros2time(node, 'now');
    cmdMsg.twist.linear.x = vlinear;
    cmdMsg.twist.angular.z = omega;
    send(cmdPub, cmdMsg);

    % Use the odometry timestamp to pair each command with its measured pose.
    odomTime = double(odomMsg.header.stamp.sec) + ...
        double(odomMsg.header.stamp.nanosec) * 1e-9;
    waypointCommands(end + 1, :) = [odomTime, vlinear, omega];
end
end
