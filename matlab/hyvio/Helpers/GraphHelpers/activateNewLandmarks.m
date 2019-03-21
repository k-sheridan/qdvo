function [graph] = activateNewLandmarks(graph)
%ACTIVATENEWLANDMARKS finds inactive landmarks in active keyframes which are well initialized
%and will cover more of the first frame in the graph.

s = Settings();
rad = s.featureSeparation;

% first sweep the frame list for all active keyframes (oldest to newest)
% and project the active landmarks in those keyframe into the newest frame,
% and put them on a spatial mask.
activeKeyframeIndices = [];

[m,n] = size(graph.FrameContainer{end}.raw_image);
spatialMask = zeros(m, n);

[idArray, activeVisible] = computeVisibleLandmarks(graph.FrameContainer{end}, graph, true, false);

for idx = (1:length(idArray))
    l = graph.getLandmark(idArray{idx}{1}, idArray{idx}{2});
    if l.status == LandmarkStatus.ACTIVE
        % mark the spatial mask
        px = activeVisible{idx};
        spatialMask(max(px(2)-rad, 1):min(px(2)+rad, m), max(px(1)-rad, 1):min(px(1)+rad, n)) = 1;
    end
end

% now sweep again, but activate landmarks which are in new areas.
error('ndofjasldjfhalksjdf')   
end
