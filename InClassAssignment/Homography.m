clear all
clc
close all;

RGB = imread("BookOriginal.jpg");
RGB2 = imresize(RGB, [640 NaN]); %Nan makes scaling proportional
imshow(RGB2);
h = gca; 
h.Visible = 'on';

% RGB3 is the rectified image

RGB3 = 0*RGB2;

% Select points on original image (unrectified)
px = [122 484 988 645];
py = [199 99 267 552];

% Select corresponding points on rectified image 
qx = [200 600 600 200];
qy = [100 100 600 600];


% H maps points from original image to the rectified image
H = myhomography(px, py, qx, qy);

% HT goes from the rectified image to the orignal image
HT = inv(H);

% Start on rectified image, use HT to find location on the original image,
% steal the color, and plug it back into the pixel from the original image


for row = 100:600
    for col = 200:600
        p_tilde = HT * [col row 1]';

        % coordinates of point in the original image
        u_original  = ceil(p_tilde(1)/p_tilde(3));
        v_original  = ceil(p_tilde(2)/p_tilde(3));

        % get the color of the pixel
        color = RGB2(v_original, u_original, :);

        % Set the color of the recctified image
        RGB3(row, col, :) = color;
    end
end

figure;
imshow(RGB3)

