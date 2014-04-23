%% Demo for how to use simulation
%% Initialization
clear all;clc;close all;
model = Quadrotor_Model;
model.initialize;
% initial position
model.setPosition(0, 0, 0);
controller = Quadrotor_Controller;
controller.setDeltaT(0.01);
controller.setConvergeErrorThreshold(0.05);
viz = Quadrotor_Visualizer;
viz.setProperties(5, 0.6);
% simulation parameters
viz_3d = 0;
viz_error = 0;
viz_interval = 3;

%% Waypoints following by Regulation (Hover control)
controller.setControlMode(2);
% Two waypoint
% waypoints = [0 0 0.5 0; 1 1 1 0];
% Randomly choose 5 waypoints
waypoints = [0 0 1 0; 0.5 0 1.5 0; 0.5 0.5 2 0; 1 0.5 1.5 0; 1 1 2 0];
% Following A line
% waypoints = [linspace(0.5, 2, 50)' linspace(0.5, 2, 50)' repmat([0.5 0], 50, 1)];
% Following A Circle

% start waypoint following
[state_data] = quadrotor_track_waypoints(model, controller, waypoints, ...
    viz, viz_3d, viz_error, viz_interval);
% visualize results in 3D
figure;hold on;grid on;
% desired straight line trajectory
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'r');
% real trajectory
plot3(state_data(:, 1), state_data(:, 2), state_data(:, 3), 'b');

%% Waypoints following by Trajectory Following
controller.setControlMode(2);
% one waypoint
waypoints = [0 0 1 0; 1 1.5 2.5 0; 2 1 3 0; 3 3 3 0; 4 5 2 0];
% Randomly choose 5 waypoints
% waypoints = [0 0 1 0; 0.5 0 1.5 0; 0.5 0.5 2 0; 1 0.5 1.5 0; 1 1 2 0];
% Following A line
% waypoints = [linspace(0.5, 2, 50)' linspace(0.5, 2, 50)' repmat([0.5 0], 50, 1)];
% Time for each segment of trajectory
timepoints = [0 1.0 2.0 3.0 4.0];
% Calculate Polynomial Function Trajectory using Quadratic Programming

% Simple Version
% t0 = 0;
% t1 = t0 + time_segments(1);
% coefficient for x, y, z, yaw
% format : [cx3 cx2 cx1 cx0 cy3 cy2 cy1 cy0 cz3 cz2 cz1 cz0 cyaw3 cyaw2 cyaw1 cyaw0]
% for x, y: 0s at 0, 1.0s at 1.0
% trajectory_coef = [1 0 0 0 1 0 0 0 0.5 0 0 0.5 0 0 0 0];
time_factor = 1;
number_order = 5;
mu_r = 1;
mu_yaw = 1;
trajectory_coef = getTrajectoryCoef(waypoints, timepoints, ...
    number_order, mu_r, mu_yaw, time_factor);
% trajectory_coef = trajectory_coef';
% start trajectory following
[state_data] = quadrotor_track_trajectory(model, controller, waypoints, ...
    trajectory_coef, timepoints, viz, viz_3d, viz_error, viz_interval);

% visualize results in 3D
figure;hold on;grid on;
% desired straight line trajectory
trajectory = [];
trajectory_x = [];
trajectory_y = [];
trajectory_z = [];
for seg_id = 2 : length(timepoints)
    t = timepoints(seg_id - 1) : controller.dt : timepoints(seg_id);
    t = [repmat(1, 1, size(t, 2)); t; t .^ 2; t .^ 3; t .^ 4; t .^ 5];
    coefficient = reshape(trajectory_coef(:, seg_id - 1), 4, number_order + 1);
    if length(trajectory_x) == 0
        trajectory_x = (coefficient(1, :) * t)';
        trajectory_y = (coefficient(2, :) * t)';
        trajectory_z = (coefficient(3, :) * t)';
        continue;
    end
    trajectory_x = [trajectory_x' coefficient(1, :) * t]';
    trajectory_y = [trajectory_y' coefficient(2, :) * t]';
    trajectory_z = [trajectory_z' coefficient(3, :) * t]';
end
trajectory = [trajectory_x trajectory_y trajectory_z];
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'g');
plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), 'r');
% real trajectory
plot3(state_data(:, 1), state_data(:, 2), state_data(:, 3), 'b');








