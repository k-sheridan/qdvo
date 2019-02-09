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
    
    % get the indices to find and modify the landmark
    [frameIdx] = graph.getFrameIndex(visibleLandmarkIDs{idx}{1});
    [landmarkIdx] = graph.FrameContainer{frameIdx}.getLandmarkIndex(visibleLandmarkIDs{idx}{2});
    
    % warp source patch to the given frame for evaluation.
    [warpedPatch, error] = warpPatchToTargetFrame(graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}, ...
        graph.FrameContainer{frameIdx}, frame, s.patchHalfSize);
    
    if error
        disp('patch warp failed, skipping.');
        continue;
    end
    
    %imagesc(warpedPatch.image, [0, 2^16])
    %colormap gray
    %drawnow;
    
    % TODO compute the prior and make the window dynamically sized.
    centerPx = visibleLandmarkPixelPositions{idx};
    
    potentialCorrespondences = {};
    
    for dx = (-s.searchRadius:s.searchRadius)
        for dy = (-s.searchRadius:s.searchRadius)
            
        end
    end
    
end

end

