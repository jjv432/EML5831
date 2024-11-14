clc;
clear all;
close all;
format compact;

%% Initial ORB for each place

% Room

room = imread("recognitionImages/room1.jpg");
room2 = imresize(room, [640 NaN]); %Nan makes scaling proportional
room2 = im2gray(room2);

% Living Room

living_room = imread("recognitionImages/livingRoom1.jpg");
living_room2 = imresize(living_room, [640 NaN]); %Nan makes scaling proportional
living_room2 = im2gray(living_room2);

% Kitchen

kitchen = imread("recognitionImages/kitchen1.jpg");
kitchen2 = imresize(kitchen, [640 NaN]); %Nan makes scaling proportional
kitchen2 = im2gray(kitchen2);

% Classroom

classroom = imread("recognitionImages/classroom1.jpg");
classroom2 = imresize(classroom, [640 NaN]); %Nan makes scaling proportional
classroom2 = im2gray(classroom2);

% Lab

lab = imread("recognitionImages/lab1.jpg");
lab2 = imresize(lab, [640 NaN]); %Nan makes scaling proportional
lab2 = im2gray(lab2);

% Compile List of all of the images
grayImages = cat(3, room2, living_room2, kitchen2, classroom2, lab2);


%% Comparing images
bestImageIndex = 0;
maxIndexPairs = 0;
imageList = ["room1.jpg", "livingRoom1.jpg", "kitchen1.jpg", "classroom1.jpg", "lab1.jpg"];
locationNames = ["Bedroom", "LivingRoom", "Kitchen", "Classroom", "Lab"];

% Generate the test image

testImage = imread("recognitionImages/lab2.jpg");
testImage2 = imresize(testImage, [640 NaN]); %Nan makes scaling proportional
testImage2 = im2gray(testImage2);

testPointsOrb = detectORBFeatures(testImage2);
testPointsSurf = detectSURFFeatures(testImage2);

% ORB Comparison
for i = 1:length(imageList)
    
    curImage = imread(strcat("recognitionImages/", imageList(i)));
    curImage2 = imresize(curImage, [640 NaN]); %Nan makes scaling proportional
    curImage2 = im2gray(curImage2);
    curImagePoints = detectORBFeatures(curImage2);
    curImage = grayImages(:, :, i);

    % Comparing current 'stock image' to the test image
    [features1,valid_points1] = extractFeatures(curImage,curImagePoints);
    [features2,valid_points2] = extractFeatures(testImage2,testPointsOrb);

    indexPairs = matchFeatures(features1,features2);

    if numel(indexPairs) > maxIndexPairs
        maxIndexPairs = numel(indexPairs);
        bestImageIndex = i;
        matchedPoints1 = valid_points1(indexPairs(:,1),:);
        matchedPoints2 = valid_points2(indexPairs(:,2),:);
        matchImage = curImage;
    end

end

fprintf("Using ORB, the test image looks like the %s\n", locationNames(bestImageIndex))


figure;
showMatchedFeatures(matchImage,testImage2,matchedPoints1,matchedPoints2);
drawnow;
title("Closest Matching Image Using ORB")


maxIndexPairs = 0;
bestImageIndex = 0;

% SURF Comparison
for i = 1:length(imageList)
    
    curImage = imread(strcat("recognitionImages/", imageList(i)));
    curImage2 = imresize(curImage, [640 NaN]); %Nan makes scaling proportional
    curImage2 = im2gray(curImage2);
    curImagePoints = detectORBFeatures(curImage2);
    curImage = grayImages(:, :, i);

    % Comparing current 'stock image' to the test image
    [features1,valid_points1] = extractFeatures(curImage,curImagePoints);
    [features2,valid_points2] = extractFeatures(testImage2,testPointsOrb);

    indexPairs = matchFeatures(features1,features2);

    if numel(indexPairs) > maxIndexPairs
        maxIndexPairs = numel(indexPairs);
        bestImageIndex = i;
        matchedPoints1 = valid_points1(indexPairs(:,1),:);
        matchedPoints2 = valid_points2(indexPairs(:,2),:);
        matchImage = curImage;
    end

end

fprintf("Using SURF, the test image looks like the %s\n", locationNames(bestImageIndex))


figure;
showMatchedFeatures(matchImage,testImage2,matchedPoints1,matchedPoints2);
drawnow;
title("Closest Matching Image Using SURF")
