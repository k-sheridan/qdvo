function [visibleLandmarkIDs, visibleLandmarkPixelPositions] = computeVisibleLandmarks(frame, graph)
%This functions computes the landmarks visible in a frame given the graph.
% visibleLandmarkIDs: vector of landmark ids
% visibleLandmarkPixelpositions: {[x1;y1], [x2;y2], ....}

assert(frame.ID > 0);

maximumKeyframes = 10;

keyframesChecked = 0;

for frameIdx = (length(graph.FrameContainer):-1:1)
    if graph.FrameContainer{frameIdx}.isKeyframe
        for landmarkIdx = (1:length(graph.FrameContainer{frameIdx}.landmarks))
            % transform and project landmark into the frame given
            assert(graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.frameID == graph.FrameContainer{frameIdx}.ID);
            assert(graph.FrameContainer{frameIdx}.ID > 0);
            
            
        end
    end 
    
    if keyframesChecked >= maximumKeyframes
        break;
    end
end

end

