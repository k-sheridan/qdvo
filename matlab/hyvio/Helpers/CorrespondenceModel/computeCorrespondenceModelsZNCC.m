function [landmarkObservationArray, graph] = computeCorrespondenceModelsZNCC(frame, graph)
%COMPUTECORRESPONDENCEMODELS generates the correspondence distributions for
% the given frame.

fprintf('computing correspondence models for frame: %i\n', frame.ID);

% this function must be optimized. it is currently ~4s for 200 features.
% This will be less in cpp, but still.



%create a settings instance
s = Settings();

% first compute the potentially observable landmarks.
[visibleLandmarkIDs, visibleLandmarkPixelPositions] = computeVisibleLandmarks(frame, graph, true, true);

landmarkObservationArray = cell(1, length(visibleLandmarkPixelPositions));

% iterate through all potentially visibl landmarks, and generate potential
% correspondences.
% in practice (c++), this could/should be done multithreaded to exploit
% multiple cores.
for idx = (1:length(visibleLandmarkIDs))
    
    % get the indices to find and modify the landmark
    [frameIdx] = graph.getFrameIndex(visibleLandmarkIDs{idx}{1});
    [landmarkIdx] = graph.FrameContainer{frameIdx}.getLandmarkIndex(visibleLandmarkIDs{idx}{2});
    
    % skip this landmark if it is not active.
    if graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.status ~= LandmarkStatus.ACTIVE
        continue;
    end
    
    % warp source patch to the given frame for evaluation.
    try
        [warpedPatch] = warpPatchToTargetFrame(graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}, ...
            graph.FrameContainer{frameIdx}, frame, graph, s.patchHalfSize);
    catch e
        fprintf('Patch Warp Failed: %s\n', e.message);
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
            catch e
                %fprintf('Patch Create Failed: %s\n', e.message);
                continue;
            end
            
            % compare patches
            try
                [score] = pm.zncc(warpedPatch, patch);
            catch e
                fprintf(1,'zncc error! The message was:%s\n',e.message);
                continue;
            end
            
            if score > s.minimumNormalizedMatchCorrelation
                %add the potential correspondence.
                pc = PotentialCorrespondence();
                pc.pixel = centerPx + [dx;dy];
                pc.score = (score - s.minimumNormalizedMatchCorrelation) / (1 - s.minimumNormalizedMatchCorrelation);
                potentialCorrespondences{end+1} = pc;
            end
            
        end
    end
    
    
    % check if this landmark had a failed correspondence
    if isempty(potentialCorrespondences)
        graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.failedCorrespondenceCounter = graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.failedCorrespondenceCounter + 1;
    else
        % reset the counter if the feature was reobserved.
        graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.failedCorrespondenceCounter = 0;
    end
    
    
    % create a landmark observation for this landmark and frame
    lo = LandmarkObservation();
    lo.potentialCorrespondenceSet = potentialCorrespondences;
    lo.landmarkParentFrameID = visibleLandmarkIDs{idx}{1};
    lo.landmarkID = visibleLandmarkIDs{idx}{2};
    lo.observationFrameID = frame.ID;
    lo.theta = s.minimumNormalizedMatchCorrelation;
    
    % debug
    lo.searchPatch = warpedPatch;
    
    
    % add to the returned array
    landmarkObservationArray{idx} = lo;
    
end

landmarkObservationArray = landmarkObservationArray(~cellfun('isempty',landmarkObservationArray)); % clear the empty cells


end

