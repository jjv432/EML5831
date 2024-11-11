clc;
clear all;
close all;
format compact;

COE = imread("CornerCOE.jpg");
COE2 = imresize(COE, [640 NaN]); %Nan makes scaling proportional
COE2 = im2double(COE2);

x_vals = 1:size(COE2, 1);
y_vals = 1:size(COE2, 2);


intensityFunction = @(x, y) ((2 * COE2(x, y, 1) + COE2(x, y, 2) + 3* COE2(x, y, 3))/6);
f = @(x)[x(1)^2 + x(2)^2, x(1)^3.*x(2)^3];
A = [1 1]

% [x, ~, ~, ~, ~, ~, jac] = lsqnonlin(intensityFunction, [1 1] ,[],[], optimset('MaxFunEvals', 0, 'MaxIter', 0));
[x, ~, ~, ~, ~, ~, jac] = lsqnonlin(f, A,[],[], optimset('MaxIter', 0, 'MaxFunEvals', 0));
jac

intensityMap = getIntensity(COE2);

figure
surfc(intensityMap);
title("Itensity Image");

J = jacobian(intensityMap, 1:size(COE2, 1));

% function intensityMap = getIntensity(image)
% 
% % https://stackoverflow.com/questions/596216/formula-to-determine-perceived-brightness-of-rgb-color
% x_length = size(image, 1);
% y_length = size(image, 2);
% 
% intensityMap = zeros(x_length, y_length);
% 
% for x = 1:x_length
%     for y = 1:y_length
%         RGB = image(x, y, :);
% 
%         R = RGB(1);
%         G = RGB(2);
%         B = RGB(3);
% 
%         % intensityMap(x,y) = (R+R+B+G+G+G)/6;
%         % intensity = (R+R+B+G+G+G)/6;
% 
%         f = @(R, G , B) (R+R+B+G+G+G)/6;
% 
%         [x, ~, ~, ~, ~, ~, jac] = lsqnonlin(f, [R, G, B] ,[],[], optimset('MaxFunEvals', 0), optimset('MaxIter', 0));
%         jac
%     end
% end
% 
% 
% end