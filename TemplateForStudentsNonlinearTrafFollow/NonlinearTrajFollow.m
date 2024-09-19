clear all
clc
close all
format compact


% Robot Initial Condition

robot.X = 0.0;
robot.Y = 0.0;
robot.Phi = 0;


%% Trajectory Generation Module

t = 0;
dt = 0.1;
t_end = 10;

t_vec = 0:dt:t_end;

for i = 1:numel(t_vec),

        if (t_vec(i)< 5)
            v(i) = 0.1*t_vec(i);
        else

            v(i) = v(i-1);
        end
        
        w(i) = 1*exp(-0.2*t_vec(i));
        
end

figure(1)
subplot(2,1,1)
    plot(t_vec,v);
    ylabel('v')
    xlabel('t')
    grid on
subplot(2,1,2)
    plot(t_vec,w);
    grid on;
    ylabel('\omega')
    xlabel('t')
    

%% Animation
figure()
grid on
% Define Robot

robot.wheelR = (13.5/2)/100; % wheel radius in m
robot.halfWidth = 0.15; % half wifth in m
robot.length = 30/100; % length in m
robot.type = 'Differential';

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
robotBody = rectangle('Parent',hgTX,'Position',[leftCornerBodyX leftCornerBodyY robot.length robot.bodyWidth],'FaceColor',[1 0 0])
shaftLength = 3.5/100;
leftCornerRightWheelX = leftCornerBodyX - (3/100);
leftCornerRightWheelY = leftCornerBodyY - shaftLength - robot.wheelWidth;

leftCornerLeftWheelX = leftCornerBodyX - (3/100);
leftCornerLeftWheelY = leftCornerBodyY + robot.bodyWidth + shaftLength;

wheelRight = rectangle('Parent',hgTX,'Position',[leftCornerRightWheelX leftCornerRightWheelY  2*robot.wheelR robot.wheelWidth],'FaceColor',[0 0 0])
wheelLeft = rectangle('Parent',hgTX,'Position',[leftCornerLeftWheelX leftCornerLeftWheelY 2*robot.wheelR robot.wheelWidth],'FaceColor',[0 0 0])


axis('equal')
axis([-4 4 -4 4])

drawRobot(hgTX,robot)

%% Simulate Robot Following Desired Trajectory

robot_history = zeros(numel(t_vec),3);

for i = 1:numel(t_vec),
    
    robot_history(i,:) = [robot.X robot.Y robot.Phi]; % desired values
    robot.vel = v(i);
    robot.angVel = w(i);
    robot = fwdSim(robot,dt);
    des_trajectory(i,:) = [v(i) w(i) robot.X robot.Y robot.Phi]; 
    drawRobot(hgTX,robot);
    pause(0.1);
   
end

hold on
plot(des_trajectory(:,3), des_trajectory(:,4),'.b')

%% Let's simulate an actual (noisy) robot

robot.X = 0.0;
robot.Y = 0.0;
robot.Phi = 0;


for i = 1:numel(t_vec),
    
           
    robot_history_actual(i,:) = [robot.X robot.Y robot.Phi]; % desired values

    % noise
    robot.vel = v(i) + 0.1*rand(1);
    robot.angVel = 0.8*w(i) + (-0.3 + 0.6*rand(1));
    %

    robot = fwdSim(robot,dt);
    %s_trajectory(i,:) = [v(i) w(i) robot.X robot.Y robot.Phi]; 
    drawRobot(hgTX,robot);
    pause(0.1);
      
   
end

hold on
plot( robot_history_actual(:,1),  robot_history_actual(:,2),'.r')


%% Your code here. Implement the nonlinear controller to track the desired trajectory
% Keep in mind that when you command the robot based on the computed
% control law, you should still assume that is noisy. Use the same type of
% noise as the one used to simulate the noisy robot

% Run your simulation and add the actual path followed by the robot.
robot.X = 0;
robot.Y = 0;
robot.Phi = 0;


for i = 1:numel(t_vec)

    % compute global errors
    Xd = des_trajectory(i,3);
    Yd = des_trajectory(i,4);
    Phi_d = des_trajectory(i,5);

    Xe = robot.X - Xd;
    Ye = robot.Y - Yd;
    Phi_e = robot.Phi - Phi_d;
    Phi_e = atan2(sin(Phi_e), cos(Phi_e));

    % transform errors to d frame (frame on trajectory) to get local errors
    correction_matrix = [cos(Phi_d) sin(Phi_d) 0; -sin(Phi_d), cos(Phi_d) 0; 0 0 1];

    T = correction_matrix*[Xe;Ye; Phi_e];
    xe = T(1);
    ye = T(2);
    phi_e = T(3);

    % define control gains
    k1 = 1;
    k2 = 3;
    k3 = 15;


    % Apply corrections
    vd = des_trajectory(i,1);
    wd = des_trajectory(i,2);

    v_control = vd - k1*abs(vd)*(xe + ye*tan(phi_e))/cos(phi_e);
    w_control = wd - (k2*vd*ye + k3*abs(vd)*tan(phi_e))*(cos(phi_e))^2;
    
    % store current robot state
    robot_history_actual(i,:) = [robot.X robot.Y robot.Phi]; % desired values

    % the robot is still noisy
    robot.vel = v_control + 0.1*rand(1);
    robot.angVel = 0.8*w_control + (-0.3 + 0.6*rand(1));

    robot = fwdSim(robot,dt);
    drawRobot(hgTX,robot);
    pause(0.1);

end

hold on
plot( robot_history_actual(:,1),  robot_history_actual(:,2),'.g')