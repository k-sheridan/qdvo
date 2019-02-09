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
[~, numFeatures] = size(newFeatures);

for index = (1:numFeatures)
    [u, error, ~] = frame.cameraModel.unproject(newFeatures(1:2, index));
    
    if error
        disp('not adding landmark');
        continue;
    end
    
    l = Landmark();
    l.frameID = frame.ID;
    l.ID = length(frame.landmarks) + 1; % create a unique landmark ID.
    
    l.bearing = u(1:2);
    l.px = newFeatures(1:2, index);
    
    % add unproject jacobian?
    
    l.dinv = 1; % initial z inverse 
    l.dinvPriorUncertainty = 1e12; % initially unknown
    
    l.patchNormal = [0;0;-1]; % best guess is that it is facing the camera.
    
    % finally, add the landmark.
    frame.landmarks{end+1} = l;
    
end

frame.isKeyframe = true;

end

