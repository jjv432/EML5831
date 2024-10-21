% Pure Pursuit Implementation
% Template for Students
clear all
clc
close all

% Robot Initial Configuration
robot.X = 0.0;
robot.Y = 0.0;
robot.Phi = 0;
robot.vel = 0.1;
dt = 0.1; % simulation step in sec

% Initialize a pure pursuit structure
purepursuit.closestWayPointIndex = 1;
purepursuit.proximityTh = 0.1; % a Threshold to determine if robot is at the waypoint

purepursuit.goalPointIndex = 1;
purepursuit.lookahead = 0.3;

% Draw robot at the initial pose
robot.wheelR = (13.5/2)/100; % wheel radius in m
robot.halfWidth = 0.15; % half width in m
robot.length = 30/100; % length in m

% Robot Geometry for Display
robot.bodyWidth = 18.5/100; % in m
robot.wheelWidth = 6.0/100; % in m; this includes the shaft

% robot Global Pose
robot.X = 0; % X Global robot position in m
robot.Y = 0; % Y Global robot position in m
robot.Phi = 0;% Global orientation in radians, measured with respect to X axis

%% Draw Robot at initial pose
leftCornerBodyX = -(robot.length/2);
leftCornerBodyY = -(robot.bodyWidth/2);

% Parent a rectangle to an hgtransform
hgTX = hgtransform;
robotBody = rectangle('Parent',hgTX,'Position',[leftCornerBodyX leftCornerBodyY robot.length robot.bodyWidth],'FaceColor',[1 0 0]);

shaftLength = 3.5/100;
leftCornerRightWheelX = leftCornerBodyX - (3/100);
leftCornerRightWheelY = leftCornerBodyY - shaftLength - robot.wheelWidth;
leftCornerLeftWheelX = leftCornerBodyX - (3/100);
leftCornerLeftWheelY = leftCornerBodyY + robot.bodyWidth + shaftLength;
wheelRight = rectangle('Parent',hgTX,'Position',[leftCornerRightWheelX leftCornerRightWheelY 2*robot.wheelR robot.wheelWidth],'FaceColor',[0 0 0]);

wheelLeft = rectangle('Parent',hgTX,'Position',[leftCornerLeftWheelX leftCornerLeftWheelY 2*robot.wheelR robot.wheelWidth],'FaceColor',[0 0 0]);

axis([-5 5 -5 5])
axis('equal')
drawRobot(hgTX,robot)

% Set of waypoints to be followed
waypoints = ginput(6);

% Plot the waypoints
hold on
plot(waypoints(:,1),waypoints(:,2),'o','MarkerSize',5,'Linewidth',2);

% Compute distance to last waypoint.
[NwayPoints, dim] = size(waypoints);
distanceRobotFinalPoint = getEuclideanDistance(robot.X,robot.Y,waypoints(NwayPoints,1),waypoints(NwayPoints,2));

%Main Loop
while((purepursuit.closestWayPointIndex <= NwayPoints) && (distanceRobotFinalPoint >= purepursuit.proximityTh))

    %Find the waypoint to head towards.
    purepursuit = findGoalWayPoint(purepursuit,robot,waypoints);

    %Make an update to the closestWaypoint so that in the next iteration the search is done starting from the latest goalPointIndex.
    purepursuit.closestWayPointIndex = purepursuit.goalPointIndex;

    % Compute the required turn radius
    [flag, turnRadius] = findTurnRadius(robot,purepursuit,waypoints);

    % Compute robot commands. The commands depend on the flag value
    % **** ADD your code here ***


    % Simulate Forward.
    robot = fwdSim(robot,dt);

    % Draw your robot at the new pose
    drawRobot(hgTX,robot);
    pause(0.01);
    
    % Compute distance to Final point to determine if robot should stop
    distanceRobotFinalPoint = getEuclideanDistance(robot.X,robot.Y,waypoints(NwayPoints,1),waypoints(NwayPoints,2));

end

%% Functions 
%% robot animation function

function drawRobot(hgTX,robot)

hgTX.Matrix = makehgtform('translate',[robot.X robot.Y 0],'zrotate',robot.Phi);

end

function robot = fwdSim(robot,dt)
    robot.X = robot.X + robot.vel*cos(robot.Phi)*dt;
    robot.Y = robot.Y + robot.vel*sin(robot.Phi)*dt;
    robot.Phi = robot.Phi + robot.angVel*dt;
end

%% Functions that you need to implement
function distance = getEuclideanDistance(x1,y1,x2,y2)

    distance = sqrt((x2-x1)^2 + (y2-y1)^2);

end

function purepursuit = findGoalWayPoint(purepursuit,robot,waypoints)
% Find the goal waypoint towards which the robot should drive. You
% must find the index of the goal waypoint that is at least one lookahead
% distance from the robot position. Note that purepursuit has a
% closestWayPointIndex that we use to always search forward.
% this function will update the field purepursuit.goalPointIndex
end

function [flag, turnRadius] = findTurnRadius(robot,purepursuit,waypoints)
% This function computes the turn radius of the circle which
% will take the robot to the goal waypoint.
% The flag output is zero if the goal is in front of the robot.
% -1 : if a left point turn should be performed
%     1: if a right hand turn should be performed
end