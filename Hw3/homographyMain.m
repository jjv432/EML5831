clc; clear all; close all; format compact;


COE = imread("CornerCOE.jpg");
COE2 = imresize(COE, [640 NaN]); %Nan makes scaling proportional
imshow(COE2);
h1 = gca; 
h1.Visible = 'on';

GWB = imread("GirlWithBalloon.jpg");
GWB2 = imresize(GWB, [320 NaN]); %Nan makes scaling proportional
imshow(GWB2);
h2 = gca; 
h2.Visible = 'on';


% Points on graffiti
px = [0 size(GWB2, 2) size(GWB2, 2) 0];
py = [0 0 size(GWB2, 1) size(GWB2, 1)];

% Points on wall
qx = [363 403 403 363];
qy = [273 252 295 311];

% H maps points from p to q
H = myhomography(px, py, qx, qy);

% Translation from the graffitti to the COE
HT = inv(H);

% Start on rectified image, use HT to find location on the original image,
% steal the color, and plug it back into the pixel from the original image

close all

for row = 1:size(GWB2, 1)
    for col = 1:size(GWB2, 2)
        p_tilde = H * [col row 1]';
    
        % get the color of the pixel
        color = GWB2(row, col, :);

        % coordinates of point in the COE image
        u_COE  = ceil(p_tilde(1)/p_tilde(3));
        v_COE  = ceil(p_tilde(2)/p_tilde(3));

         
        % Set the color of the recctified image
        COE2(v_COE, u_COE, :) = color;
        
    end
    
end

figure;
imshow(COE2)

