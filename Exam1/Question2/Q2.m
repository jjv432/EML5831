clc
clear
close all
format compact

%% Create robot parameters 
Body.x = 0;
Body.y = 0;
Body.phi = pi/2;
Body.radius = 1;
Body.width = 1;
Body.length = .5;

Wheel.radius = .125;
Wheel.wheel_width = 0.125;

%% Creating Obstructions

CircleObst.Location = [6, 3];
CircleObst.Radius = 2;

SquareObst.Location = [2,6];
SquareObst.Height = 1;
SquareObst.Length = 1;

RectangleObst.Location = [-5 0];
RectangleObst.Height = 3;
RectangleObst.Length = 2;

[Circle, Square, Rectangle] =drawObstructions(CircleObst, SquareObst, RectangleObst);

%% LIDAR

Max_Range = 5;
Min_Range = .2;
Range_Resolution = .01;

% Search range




%% Create Trajectory (AXE?)

sample_num = 1000;
sample_length = 2;
circle_radius = 3;

x_vals = zeros(sample_num,1) + Body.x;
y_vals = (linspace(0, sample_length, sample_num)) + Body.y;

%% Plot the Robot


Body.phi = pi/2;

for i = 1:length(x_vals)
    Body.x = x_vals(i);
    Body.y = y_vals(i);
    [h1, h2, h3] = drawRobot(Body, Wheel);

    thetas = linspace(0, 2*pi, 50);

    SearchOuterX_Vals = Max_Range * cos(thetas) + Body.x;
    SearchOuterY_Vals = Max_Range * sin(thetas) + Body.y;
    
    SearchInnerX_Vals = Min_Range * cos(thetas);
    SearchInnerY_Vals = Min_Range * sin(thetas);


    % Doing q and v wrong?
    xq = [Square.XVals, Circle.XVals, Rectangle.XVals];
    yq = [Square.YVals, Circle.YVals, Rectangle.YVals];

    xv = [SearchOuterX_Vals, NaN(1,1), (SearchInnerX_Vals)];
    yv = [SearchOuterY_Vals, NaN(1,1), (SearchInnerY_Vals)];

    [in, on] = inpolygon(xq, yq, xv, yv);
    h4 = plot(xq(in),yq(in),'bx');
    h5 = plot(SearchOuterX_Vals(~in(1:length(SearchOuterX_Vals))), SearchOuterY_Vals(~in(1:length(SearchOuterY_Vals))), 'bx');
    
    
    pause(0)
    delete(h1);
    delete(h2);
    delete(h3);
    delete(h4);
    delete(h5);


end
%%
%
% <include>drawRobot.m</include> 
%