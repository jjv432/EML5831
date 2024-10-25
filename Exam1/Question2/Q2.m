clc
clear
close all
format compact

%% Create robot parameters 
Body.x = 0;
Body.y = 1;
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

sample_num = 10;
sample_length = .5;
circle_radius = 3;

x_vals = zeros(sample_num,1) + Body.x;
y_vals = (linspace(0, sample_length, sample_num)) + Body.y;

%% Plot the Robot


Body.phi = pi/2;
thetas = 0:(pi/180):2*pi;

for i = 1:sample_num

    Body.x = x_vals(i);
    Body.y = y_vals(i);
    [h1, h2, h3] = drawRobot(Body, Wheel);

   
    SearchOuterX_Vals = Max_Range * cos(thetas) + Body.x;
    SearchOuterY_Vals = Max_Range * sin(thetas) + Body.y;
    
    SearchInnerX_Vals = Min_Range * cos(thetas) + Body.x;
    SearchInnerY_Vals = Min_Range * sin(thetas) + Body.y;


    xq = [SearchOuterX_Vals, NaN(1,1), (SearchInnerX_Vals)];
    yq = [SearchOuterY_Vals, NaN(1,1), (SearchInnerY_Vals)];


    for a = 1:length(thetas)
        x(a) = xq(a);
        y(a) = yq(a);

        inCircle  = inpolygon(x(a), y(a), Circle.XVals, Circle.YVals);
        inRectangle = inpolygon(x(a), y(a), Rectangle.XVals, Rectangle.YVals);
        inSquare = inpolygon(x(a), y(a), Square.XVals, Square.YVals);

        while (inCircle)
            x(a) = x(a)- Range_Resolution;% * cos(a);
            y(a) = y(a)- Range_Resolution;% * sin(a);
            inCircle  = inpolygon(x(a), y(a), Circle.XVals, Circle.YVals);
        end

        while (inRectangle)
            x(a) = x(a) +  Range_Resolution;% * cos(a);
            y(a) = y(a) + Range_Resolution;% * sin(a);
            inRectangle = inpolygon(x(a), y(a), Rectangle.XVals, Rectangle.YVals);
        end

        while (inSquare)
            x(a) = x(a) - Range_Resolution;% * cos(a);
            y(a) = y(a) - Range_Resolution;% * sin(a);
            inSquare = inpolygon(x(a), y(a), Square.XVals, Square.YVals);
        end
    end
    
    h4 = plot(x, y, 'bx');
    pause(.001)
    delete(h1)
    delete(h2)
    delete(h3)
    delete(h4)
    
end
%%
%
% <include>drawRobot.m</include> 
%