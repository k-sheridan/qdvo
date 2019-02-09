function [visibleLandmarkIDs, visibleLandmarkPixelPositions] = computeVisibleLandmarks(frame, graph)
%This functions computes the landmarks visible in a frame given the graph.
% visibleLandmarkIDs: vector of landmark ids
% visibleLandmarkPixelpositions: {[x1;y1], [x2;y2], ....}

assert(frame.ID > 0);

maximumKeyframes = 10;
minimumDepth = 0.01;
minimumCos = cosd(70);

keyframesChecked = 0;

visibleLandmarkIDs = {};
visibleLandmarkPixelPositions = {};

for frameIdx = (length(graph.FrameContainer):-1:1)
    if graph.FrameContainer{frameIdx}.isKeyframe
        
        keyframesChecked = keyframesChecked + 1;
        
        for landmarkIdx = (1:length(graph.FrameContainer{frameIdx}.landmarks))
            % transform and project landmark into the frame given
            assert(graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.frameID == graph.FrameContainer{frameIdx}.ID);
            assert(graph.FrameContainer{frameIdx}.ID > 0);
            
            % inv(T_w2c)*T_w2l*bearing/dinv
            % best estimate of landmark position in current frame
            T = inv(graph.FrameContainer{frameIdx}.imustate.poseTransform()) * ...
                frame.imustate.poseTransform();
            
            r_cam = T(1:3, 1:3) * ...
                [graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.bearing; 1] * ...
                1/graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.dinv + T(1:3, 4);
            
            if r_cam(3) < minimumDepth
                disp('potential landmark behind camera')
                continue; % skip
            end
            
            
            % perform normal check
            patchNormInCurrentFrame = T(1:3, 1:3) * graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.patchNormal;
            
            % Assume patchNorm is unit vec
            assert(abs(norm(patchNormInCurrentFrame) - 1) < 1e-6);
            
            if -1 * patchNormInCurrentFrame(3) < minimumCos
                disp('potential landmark norm not facing cam');
                continue;
            end
            
            % at this point we can call this landmark 'visible'
            
            [px, error] = frame.cameraModel.project(r_cam);
            
            if error
                disp('could not project feature into camera');
                continue;
            end
            
            visibleLandmarkIDs{end+1} = graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.ID;
            visibleLandmarkPixelPositions{end+1} = px;
            
        end
    end 
    
    if keyframesChecked >= maximumKeyframes
        break;
    end
end

end

