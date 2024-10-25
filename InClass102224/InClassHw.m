clc;
close all;
format compact;


RGB = imread("CornerCOE.jpg");
RGB2 = imresize(RGB, [640 NaN]);
imshow(RGB2)
h = gca;
h.Visible = 'on';
P1 = [264 274 1];
P2 = [619 13 1];

l1 = line_given_2_points(P1, P2);

th_1 = acosd(l1(1));
rho_1 = -l1(3);

P3 = [263 289 1];
P4 = [619 63 1];

l2 = line_given_2_points(P3, P4);
a2 = l2(1);
b2 = l2(2);
c2 = l2(3);

th_2 = acosd(a2);
rho_2 = -c2;

alpha = 0:1:90;
r1 = rho_1./cosd(th_1 - alpha);
x1 = r1.*cosd(alpha);
y1 = r1.*sind(alpha);

r2 = rho_2./cosd(th_2 - alpha);
x2 = r2.*cosd(alpha);
y2 = r2.*sind(alpha);

hold on
plot(x1, y1, 'r')
plot(x2, y2, 'r')

P = findIntersectionBetween2Lines(l1, l2);
plot(P(1), P(2), 'xb', 'LineWidth', 2, 'MarkerSize', 10)




function [l, rho, theta] = line_given_2_points(p1, p2)

l = cross(p1, p2);
l = l/(sqrt(l(1)^2 + l(2)^2));
theta = atan2(l(2), l(1));
rho = l(3);

end

function [Pint] = findIntersectionBetween2Lines(l1, l2)

Pint = cross(l1, l2);
Pint = [Pint(1)/Pint(3), Pint(2)/Pint(3)]';

end
