function [Circle, Square, Rectangle] = drawObstructions(CircleObst, SquareObst, RectangleObst)


%% Cirlce Obstruction

CircleX = CircleObst.Location(1);
CircleY = CircleObst.Location(2);

thetas = linspace(0, 2*pi, 1000);

CircleXVals = CircleObst.Radius * cos(thetas) + CircleX;
CircleYVals = CircleObst.Radius * sin(thetas) + CircleY;

Circle.XVals = CircleXVals;
Circle.YVals = CircleYVals;

%% Square Obstruction

SquareX = SquareObst.Location(1);
SquareY = SquareObst.Location(2);

Square_dY = SquareObst.Height;
Square_dX = SquareObst.Length;

maxX = SquareX + Square_dX/2;
minX = SquareX - Square_dX/2;

maxY = SquareY + Square_dY/2;
minY = SquareY - Square_dY/2;

% Need to make the values repeat themselves so I have every point on the
% boxes
SquareXVals = [minX, minX, maxX, maxX];
SquareYVals = [minY, maxY, maxY, minY];

Square.XVals = SquareXVals;
Square.YVals = SquareYVals;

%% Rectangle Obstruction

RectangleX = RectangleObst.Location(1);
RectangleY = RectangleObst.Location(2);

Rectangle_dY = RectangleObst.Height;
Rectangle_dX = RectangleObst.Length;

maxX = RectangleX + Rectangle_dX/2;
minX = RectangleX - Rectangle_dX/2;

maxY = RectangleY + Rectangle_dY/2;
minY = RectangleY - Rectangle_dY/2;

RectangleXVals = [minX, minX, maxX, maxX];
RectangleYVals = [minY, maxY, maxY, minY];

Rectangle.XVals = RectangleXVals;
Rectangle.YVals = RectangleYVals;


%% Plotting the Obstructions

figure()
hold on
grid on
fill(SquareXVals, SquareYVals, 'k');
fill(CircleXVals, CircleYVals, 'k');
fill(RectangleXVals, RectangleYVals, 'k');
axis equal
axis([-10 10 -10 10])
end