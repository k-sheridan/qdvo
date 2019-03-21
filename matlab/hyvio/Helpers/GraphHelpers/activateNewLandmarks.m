function [graph] = activateNewLandmarks(graph)
%ACTIVATENEWLANDMARKS finds inactive landmarks in active keyframes which are well initialized
%and will cover more of the first frame in the graph.

s = Settings();

% first sweep the frame list for all active keyframes (oldest to newest)
% and project the active landmarks in those keyframe into the newest frame,
% and put them on a spatial mask.
activeKeyframeIndices = [];

[m,n] = size(graph.FrameContainer{end}.raw_image);
spatialMask = zeros(m, n);

for idx = (1:length(graph.FrameContainer))
    if graph.FrameContainer{idx}.status == FrameStatus.ACTIVE
        if graph.FrameContainer{idx}.isKeyframe
            activeKeyframeIndices = [activeKeyframeIndices, idx];
            
            for l = graph.FrameContainer{idx}.landmarks
                % project the feature into the newest frame.
                
            end
            
        else
            error('Active Frame is not a keyframe.')
        end
    end
end

