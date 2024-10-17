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
old_error = des.Y - robot.Y;

% tracks total error
error_sum = 0;

% noise term
drift = 0.1;

% hand picked gains
gains = [.3 2 .05];
y_values = [];
% Simulates and draws robot positions with controller
for i = 1:500
    y_front_wheel = drawRobot_Ackerman(robot, Wheel,i);
    pause(0)
    width = robot.width;
    robot = fwdSim(robot, dt);
    [omega, gamma, error, error_sum] = my_controller(robot, des, old_error, error_sum, dt, gains, width, drift);

    Wheel.gamma = gamma;
    robot.angVel = omega;
   
    old_error = error;

    steering_angles(i) = gamma;

    y_values(i) = robot.Y;
   

end

figure()
    plot(steering_angles)
    xlabel("Iteration");
    ylabel("Steering Angle \gamma");
    title("Steering Angle vs Iteration")
figure()
    plot(y_values)
    xlabel("Iteration");
    ylabel("Cross Track Error");
    title("Cross Track Error v Time")
%%
%
% <include>drawRobot_Ackerman.m</include> 
%
% <include>my_controller.m</include>
%
% <include>fwdSim.m</include>
%