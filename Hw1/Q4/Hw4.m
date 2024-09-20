clc
clear all
close all
format compact

%Robot characteristics
robot.X = 0;
robot.Y = 10;
robot.Phi = 0;
robot.radius = 1;
robot.width = 2;
robot.length = 1;


robot.Vel = 1;
robot.angVel = 0;

Wheel.radius = .25;
Wheel.wheel_width = 0.125;

Wheel.gamma = 0;

%
des.Y = 0;
old_error = 5;

InitialRobot = robot;

G = my_PSO(InitialRobot, des, old_error, Wheel);


