clc;
clear all;
close all;
format compact;

COE = imread("CornerCOE.jpg");
COE2 = imresize(COE, [640 NaN]); %Nan makes scaling proportional
COE2 = im2double(COE2);

x_vals = 1:size(COE2, 1);
y_vals = 1:size(COE2, 2);


intensityMap = getIntensity(COE2);

syms x y;

% Jx = jacobian(intensityMap(x,y), [x])

[Tx, Ty] = gradient(intensityMap);

% The jacobian is the transpose of the gradient
Jx = transpose(Tx);
Jy = transpose(Ty);

A = 0; 
B = 0; 
C = 0; 
D = 0;

edgeIndices = [];

for u = 1:length(x_vals)
    for v = 1:length(y_vals)
        A = (Jx(v, u))^2;
        B = Jx(v, u) * Jy(v, u);
        C = Jy(v, u) * Jx(v, u);
        D = (Jy(v, u))^2;
        stucutreMatrix = [A, B; C, D];

        if D > .02
            edgeIndices = [edgeIndices; u, v];
        end
    end
end

figure()
imshow(COE2);
hold on
plot(edgeIndices, 'rx', 'MarkerSize', 10)

figure
surfc(intensityMap);
title("Itensity Image");



function intensityMap = getIntensity(image)

% https://stackoverflow.com/questions/596216/formula-to-determine-perceived-brightness-of-rgb-color
x_length = size(image, 1);
y_length = size(image, 2);

intensityMap = zeros(x_length, y_length);

for x = 1:x_length
    for y = 1:y_length
        RGB = image(x, y, :);

        R = RGB(1);
        G = RGB(2);
        B = RGB(3);
        intensity = (R+R+B+G+G+G)/6;
        intensityMap(x,y) = intensity;

    end
end

end