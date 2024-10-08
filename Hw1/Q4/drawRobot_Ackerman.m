function [y_front_wheel] = drawRobot_Ackerman(robot, Wheel)


%% Main body
length = robot.length; %y-direction
width = robot.width; % x-direction

% Body vertices
y_box = [-length/2 -length/2 length/2 length/2 -length/2];
x_box = [width 0 0 width width];

% Robot initial conditions
x = robot.X;
y = robot.Y;
phi = robot.Phi;

% Rotating and translating robot body
rot_matrix = [cos(phi), -sin(phi); sin(phi), cos(phi)];
box_rotated = rot_matrix * [x_box; y_box];

box_translated_rotated = [box_rotated(1,:) + x; box_rotated(2,:) + y];


%% Wheels
radius = Wheel.radius;
wheel_width = Wheel.wheel_width; %y direction


%Back Wheel

x_back_wheel = [(-radius), ( -radius), (radius), (radius), (-radius)];
y_back_wheel = [wheel_width, -wheel_width, -wheel_width, wheel_width, wheel_width];

back_wheel_rotated = rot_matrix * [x_back_wheel; y_back_wheel];

back_wheel_translated_rotated = [back_wheel_rotated(1,:) + x; back_wheel_rotated(2,:) + y];


%Front Wheel
gamma = Wheel.gamma;

% Initial position
x_front_wheel = [(-radius), ( -radius), (radius), (radius), (-radius)];
y_front_wheel = [wheel_width, -wheel_width, -wheel_width, wheel_width, wheel_width];

% Rotate wheel by steering angle
front_rot_matrix = [cos(gamma), -sin(gamma); sin(gamma), cos(gamma)];
front_wheel_steered = front_rot_matrix * [x_front_wheel; y_front_wheel];

% Put wheel at front of the body
front_wheel_translated = [front_wheel_steered(1,:) + width; front_wheel_steered(2,:)];

% Rotate wheel with body
front_wheel_rotated = rot_matrix * front_wheel_translated;

% Combine rotation and translation
front_wheel_translated_rotated = [front_wheel_rotated(1,:) + x; front_wheel_rotated(2,:) + y];


%% Plotting
cla
fill(box_translated_rotated(1,:), box_translated_rotated(2,:), 'b')
hold on
grid on
fill(back_wheel_translated_rotated(1,:), back_wheel_translated_rotated(2,:), 'k');
hold on
fill(front_wheel_translated_rotated(1,:), front_wheel_translated_rotated(2,:), 'k')
axis([(-12+ x) (12+x) -12 12])
drawnow;

