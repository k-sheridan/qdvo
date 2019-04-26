function [isKeyframe] = isKeyframe(graph, frameID)
%ISKEYFRAME This function implements a DSO like keyframe selection
%criteria.

isKeyframe = false;

s = Settings();

fidx = graph.getFrameIndex(frameID);

% find the newest ACTIVE keyframe.
kfIdx = -1;
for idx = (length(graph.FrameContainer):-1:1)
    if graph.FrameContainer{idx}.isKeyframe && graph.FrameContainer{idx}.status == FrameStatus.ACTIVE
        kfIdx = idx;
        break;
    end
end

% compute the visible and active landmark position in the newest keyframe
[landmarkIDs, pixelPositions] = computeVisibleLandmarks(graph.FrameContainer{kfIdx}, graph, true, true); 

% compute the average pixel flow
n = 0;
avgPixelFlow = 0;
for idx = (1:length(landmarkIDs))
    pid = landmarkIDs{idx}{1};
    lid = landmarkIDs{idx}{2};
    
    try
        [px] = projectLandmark(graph, pid, lid, graph.FrameContainer{fidx}.ID);
    catch
        fprintf('Failed to project visible feature in newest keyframe.\n');
        continue;
    end
    
    avgPixelFlow = avgPixelFlow + norm(pixelPositions{idx} - px);
    n = n + 1;
end

if n > 0
    avgPixelFlow = avgPixelFlow / n;
else
    avgPixelFlow = 0;
end
fprintf('Found average pixel flow of: %f with %i features \n', avgPixelFlow, n);


% compute the average translation only pixel flow
R_original = graph.FrameContainer{fidx}.imustate.R;
graph.FrameContainer{fidx}.imustate.R = graph.FrameContainer{kfIdx}.imustate.R;
n = 0;
avgPixelTranslationFlow = 0;
for idx = (1:length(landmarkIDs))
    pid = landmarkIDs{idx}{1};
    lid = landmarkIDs{idx}{2};
    
    try
        [px] = projectLandmark(graph, pid, lid, graph.FrameContainer{fidx}.ID);
    catch
        fprintf('Failed to project visible feature in newest keyframe.\n');
        continue;
    end
    
    avgPixelTranslationFlow = avgPixelTranslationFlow + norm(pixelPositions{idx} - px);
    n = n + 1;
end
graph.FrameContainer{fidx}.imustate.R = R_original;

if n > 0
    avgPixelTranslationFlow = avgPixelTranslationFlow / n;
else
    avgPixelTranslationFlow = 0;
end
fprintf('Found average pixel translational flow of: %f with %i features \n', avgPixelTranslationFlow, n);


if s.weightAvgPixelFlow * avgPixelFlow + s.weightAvgTranslationalFlow * avgPixelTranslationFlow > 1
    isKeyframe = true;
    fprintf('Frame is Keyframe\n');
end

end

