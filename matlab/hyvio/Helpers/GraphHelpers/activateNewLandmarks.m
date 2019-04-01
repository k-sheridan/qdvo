function [graph] = activateNewLandmarks(graph)
%ACTIVATENEWLANDMARKS finds inactive landmarks in active keyframes which are well initialized
%and will cover more of the first frame in the graph.

s = Settings();
rad = s.activationFeatureSeparation;

% first sweep the frame list for all active keyframes (oldest to newest)
% and project the active landmarks in those keyframe into the newest frame,
% and put them on a spatial mask.
activeKeyframeIndices = [];

[m,n] = size(graph.FrameContainer{end}.raw_image);
spatialMask = zeros(m, n);

[idArray, activeVisible] = computeVisibleLandmarks(graph.FrameContainer{end}, graph, true, false);

nActiveVisible = 0;

for idx = (1:length(idArray))
    l = graph.getLandmark(idArray{idx}{1}, idArray{idx}{2});
    if l.status == LandmarkStatus.ACTIVE
        % mark the spatial mask
        px = activeVisible{idx};
        spatialMask(max(px(2)-rad, 1):min(px(2)+rad, m), max(px(1)-rad, 1):min(px(1)+rad, n)) = 1;
        nActiveVisible = nActiveVisible + 1;
    end
end

%TODO gaussian blur the spatial mask to get a 'distance' score function.


% now sweep again, but activate initialized landmarks which are in new areas.
for idx =  (1:length(idArray))
    if nActiveVisible > s.nActiveLandmarks
        break;
    end
    
    
    fidx = graph.getFrameIndex(idArray{idx}{1});
    lidx = graph.FrameContainer{fidx}.getLandmarkIndex(idArray{idx}{2});
    
    if graph.FrameContainer{fidx}.landmarks{lidx}.status == LandmarkStatus.INACTIVE
        px = round(activeVisible{idx});
        try
            occupied = spatialMask(px(2), px(1));
        catch
            occupied = 1;
        end
        
        if ~occupied && graph.FrameContainer{fidx}.landmarks{lidx}.epipolarDepthEstimator.initialized
            graph.FrameContainer{fidx}.landmarks{lidx}.status = LandmarkStatus.ACTIVE;
            spatialMask(max(px(2)-rad, 1):min(px(2)+rad, m), max(px(1)-rad, 1):min(px(1)+rad, n)) = 1;
            nActiveVisible = nActiveVisible + 1;
            fprintf('Activated landmark\n');
        end
    end
    
    
end


% if we still have too few active features, we need to activate
% uninitialized landmarks

if nActiveVisible < 20
    fprintf('NEED TO ACTIVE UNINITIALIZED LANDMARKS!');
    
    % now sweep again, but activate landmarks which are in new areas.
    for idx =  (1:length(idArray))
        if nActiveVisible > s.nActiveLandmarks
            break;
        end
        
        
        fidx = graph.getFrameIndex(idArray{idx}{1});
        lidx = graph.FrameContainer{fidx}.getLandmarkIndex(idArray{idx}{2});
        
        if graph.FrameContainer{fidx}.landmarks{lidx}.status == LandmarkStatus.INACTIVE
            px = round(activeVisible{idx});
            try
                occupied = spatialMask(px(2), px(1));
            catch
                occupied = 1;
            end
            
            if ~occupied
                graph.FrameContainer{fidx}.landmarks{lidx}.status = LandmarkStatus.ACTIVE;
                spatialMask(max(px(2)-rad, 1):min(px(2)+rad, m), max(px(1)-rad, 1):min(px(1)+rad, n)) = 1;
                nActiveVisible = nActiveVisible + 1;
                fprintf('Activated uninitialized landmark\n');
            end
        end
        
        
    end
end

end
