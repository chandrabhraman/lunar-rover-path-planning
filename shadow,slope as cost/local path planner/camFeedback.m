function [obstacle, MAX_X, MAX_Y] = camFeedback()
%UNTITLED2 Summary of this function goes here
%generator function to obtain obstacles.

%determines the threshold height/width ratio to ignore in the feild map.
threshold = 15;
num = 1;

%capture frame
frameL = captureFrameL();
frameR = captureFrameR();


%detect obstacle dimensions for traversal
%feature dimension should contain
%1. x center coordinate of obstacle
%2. y center coordinate of obstacle
%3. height of obstacle.
%4. width of obstacle.

featureDim = fDim(frameL, frameR); %1.height 2.width
[MAX_X, MAX_Y] = ROIDim(frameL, frameR) %should return dimensions of the ROI in consideration.

for a=1:size(featureDim)
    if height/distance > threshold 
        obstacle(num) = featureDim(a); 
        num = num+1;
    end
end

if num == 1
    obstacle = [];
end
end