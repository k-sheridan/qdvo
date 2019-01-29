function [frame] = createNewLandmarks(frame, graph, maxFeatures)
% This function will create new landmarks for a given frame such that a
% more even feature distribution is achieved. 
%This should be the only function which has the capability of adding new
%landmarks.
% MaxFeatures - integer which specifies the desired number of visible features in
% this keyframe
% NOTE: this function does not add the frame to the graph. It only adds new
% landmarks to the vector.



%TODO compute the set of pixels which already have a visible landmark.
currentFeatures = [];
numCurrentFeatures = length(currentFeatures);

% Run feature detection
newFeatures = detectFeatures(frame.raw_image, frame.cameraModel, currentFeatures, maxFeatures-numCurrentFeatures, 50);

% Create new landmarks for each detected feature

end

