function [landmarkObservationArray] = computeCorrespondenceModels(frame, graph)
%COMPUTECORRESPONDENCEMODELS generates the correspondence distributions for
% the given frame.

%create a settings instance
s = Settings();

% first compute the potentially observable landmarks.
[visibleLandmarkIDs, visibleLandmarkPixelPositions] = computeVisibleLandmarks(frame, graph);

% iterate through all potentially visibl landmarks, and generate potential
% correspondences.
% in practice (c++), this could/should be done multithreaded to exploit
% multiple cores.
for idx = (1:length(visibleLandmarkIDs))
    
    
    
    % warp source patch to the given frame for evaluation.
    warpedPatch = warpPatchToTargetFrame()
end

end

