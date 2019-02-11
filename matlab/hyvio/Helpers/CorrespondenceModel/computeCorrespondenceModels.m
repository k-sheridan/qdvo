function [landmarkObservationArray] = computeCorrespondenceModels(frame, graph)
%COMPUTECORRESPONDENCEMODELS generates the correspondence distributions for
% the given frame.

% this function must be optimized. it is currently ~4s for 200 features.
% This will be less in cpp, but still.

landmarkObservationArray = {};

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
    centerPx = round(visibleLandmarkPixelPositions{idx});
    
    potentialCorrespondences = {};
    pm = ZNCCPatchMatcher(s);
    
    %TODO compute mean and sd over whole search area.
%     lowerBound = centerPx - s.patchHalfSize - s.searchRadius;
%     upperBound = centerPx + s.patchHalfSize + s.searchRadius;
%     lowerBound = max(lowerBound, 1);
%     upperBound = min(upperBound, frame.cameraModel.size);
%     
%     % This will compute the mean and sd over the whole window.
%     patch = Patch(frame.raw_image(lowerBound(2):upperBound(2), lowerBound(1):upperBound(1)));
%     regionMean = patch.meanIntensity;
    
    for dx = (-s.searchRadius:s.searchRadius)
        for dy = (-s.searchRadius:s.searchRadius)
            try
                [patch] = patchFromImage(frame.raw_image, centerPx + [dx;dy], s.patchHalfSize);
            catch
                disp('failed to get patch for potential correspondence.');
                continue;
            end
            
            % compare patches
            try
                [score] = pm.zncc(warpedPatch, patch);
            catch e
                fprintf(1,'zncc error! The message was:\n%s',e.message);
                continue;
            end
            
            if score > s.minimumNormalizedMatchCorrelation
                %add the potential correspondence.
                pc = PotentialCorrespondence();
                pc.pixel = centerPx + [dx;dy];
                pc.score = score;
                potentialCorrespondences{end+1} = pc;
            end
            
        end
    end
    
    % create a landmark observation for this landmark and frame
    if ~isempty(potentialCorrespondences)
        lo = LandmarkObservation();
        lo.potentialCorrespondenceSet = potentialCorrespondences;
        lo.landmarkParentFrameID = visibleLandmarkIDs{idx}{1};
        lo.landmarkID = visibleLandmarkIDs{idx}{2};
        lo.observationFrameID = frame.ID;
        
        % add to the returned array
        landmarkObservationArray{end+1} = lo;
    end
    
end

end

