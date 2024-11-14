clc;
clear all;
close all;
format compact;

% COE = imread("CornerCOE.jpg");
% COE2 = imresize(COE, [640 NaN]); %Nan makes scaling proportional
% COE2 = im2gray(COE2);
% 
% points = detectORBFeatures(COE2);
% 
% figure();
% imshow(COE2);
% hold on
% plot(points, 'ShowScale', false);

%% Initial ORB for each place

% Room

room = imread("recognitionImages/room1.jpg");
room2 = imresize(room, [640 NaN]); %Nan makes scaling proportional
room2 = im2gray(room2);

roomPoints = detectORBFeatures(room2);

% Living Room

living_room = imread("recognitionImages/livingRoom1.jpg");
living_room2 = imresize(living_room, [640 NaN]); %Nan makes scaling proportional
living_room2 = im2gray(living_room2);

living_roomPoints = detectORBFeatures(living_room2);

% Kitchen

kitchen = imread("recognitionImages/kitchen1.jpg");
ktichen2 = imresize(kitchen, [640 NaN]); %Nan makes scaling proportional
ktichen2 = im2gray(ktichen2);

kitchenPoints = detectORBFeatures(ktichen2);

% Classroom

classroom = imread("recognitionImages/classroom1.jpg");
classroom2 = imresize(classroom, [640 NaN]); %Nan makes scaling proportional
classroom2 = im2gray(classroom2);

classroomPoints = detectORBFeatures(classroom2);

%% Comparing images

% testImage = imread("recognitionImages/classroom2.jpg");
% testImage2 = imresize(testImage, [640 NaN]); %Nan makes scaling proportional
% testImage2 = im2gray(testImage2);
% 
% testImagePoints = detectORBFeatures(testImage2);
% 
% [features1,valid_points1] = extractFeatures(testImage2,testImagePoints);
% [features2,valid_points2] = extractFeatures(classroom2,classroomPoints);
% 
% indexPairs = matchFeatures(features1,features2);
% 
% matchedPoints1 = valid_points1(indexPairs(:,1),:);
% matchedPoints2 = valid_points2(indexPairs(:,2),:);
% 
% figure; 
% showMatchedFeatures(testImage2,classroom2,matchedPoints1,matchedPoints2);

maxIndexPairs = 0;
imageList = ["room1.jpg", "livingRoom1.jpg", "kitchen1.jpg", "classroom1.jpg"];

% Generate the test image

testImage = imread("recognitionImages/kitchen2.jpg");
testImage2 = imresize(testImage, [640 NaN]); %Nan makes scaling proportional
testImage2 = im2gray(testImage2);

testPoints = detectORBFeatures(testImage2);


for i = 1:length(imageList)
    
    % Organizing all the info for the current image we're comparing againts
    curImage = imread(strcat("recognitionImages/", imageList(i)));
    curImage2 = imresize(curImage, [640 NaN]); %Nan makes scaling proportional
    curImage2 = im2gray(curImage2);
    curImagePoints = detectORBFeatures(curImage2);
    
    % Comparing current 'stock image' to the test image
    [features1,valid_points1] = extractFeatures(curImage2,curImagePoints);
    [features2,valid_points2] = extractFeatures(testImage2,testPoints);

    indexPairs = matchFeatures(features1,features2);

    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);

    figure;
    showMatchedFeatures(curImage2,testImage2,matchedPoints1,matchedPoints2);

end