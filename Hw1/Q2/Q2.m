%% Format
clc
clear all
close all
format compact

%% Physical/ Initial Characteristics

%Robot characteristics
robot.X = 0;
robot.Y = 10;
robot.Phi = 0;
robot.radius = 1;
robot.width = 2;
robot.length = 1;
robot.Vel = 1;
robot.angVel = 0;

% Wheel Characteristics
Wheel.radius = .25;
Wheel.wheel_width = 0.125;
Wheel.gamma = 0;

%% Control Implementation

% Desired y value
des.Y = 0;

% simulated dt
dt = .075;

% initial error in y
old_error = 5;

% tracks total error
error_sum = 0;

% noise term
drift = 0.1;

% hand picked gains
gains = [.2 .9 .1];

% Simulates and draws robot positions with controller
for i = 1:2500
    clf
    y_front_wheel = drawRobot_Ackerman(robot, Wheel);
    pause(0)
    width = robot.width;
    robot = fwdSim(robot, dt);
    [omega, gamma, error] = my_controller(robot, des, old_error, error_sum, dt, gains, width, drift);

    Wheel.gamma = gamma;
    robot.angVel = omega;
   
    old_error = error;
   

end













% %Plotting
% 
% robot.phi = pi/2;
% for i = 1:length(x_vals)
%     clf
%     robot.x = x_vals(i);
%     robot.y = y_vals(i);
%     robot.phi = robot.phi + 2*pi/sample_length;
%     drawRobot_Ackerman(robot, Wheel);
%     pause(0)
% 
% 
% end