function [visibleLandmarkIDs, visibleLandmarkPixelPositions] = computeVisibleLandmarks(frame, graph)
%This functions computes the landmarks visible in a frame given the graph.
% visibleLandmarkIDs: vector of parent + landmark ids: {{parentFrameID, landmarkID}}
% visibleLandmarkPixelpositions: {[x1;y1], [x2;y2], ....}

assert(frame.ID > 0);

maximumKeyframes = 40;
minimumDepth = 0.01;
minimumCos = cosd(70);
maximumRadiusRatio = 0.8;

keyframesChecked = 0;

visibleLandmarkIDs = {};
visibleLandmarkPixelPositions = {};

T_i_c = graph.extrinsics.getImu2CameraTransform(frame.camID);

for frameIdx = (length(graph.FrameContainer):-1:1)
    if graph.FrameContainer{frameIdx}.isKeyframe
        
        keyframesChecked = keyframesChecked + 1;
        
        for landmarkIdx = (1:length(graph.FrameContainer{frameIdx}.landmarks))
            % transform and project landmark into the frame given
            assert(graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.frameID == graph.FrameContainer{frameIdx}.ID);
            assert(graph.FrameContainer{frameIdx}.ID > 0);
            
            % inv(T_w2c)*T_w2l*bearing/dinv
            % best estimate of landmark position in current frame
            T_target_source = (frame.imustate.poseTransform() * T_i_c) \ ...
                (graph.FrameContainer{frameIdx}.imustate.poseTransform() * T_i_c);
            
            r_cam = T_target_source(1:3, 1:3) * ...
                [graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.bearing; 1] * ...
                1/graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.dinv + T_target_source(1:3, 4);
            
            if r_cam(3) < minimumDepth
                disp('potential landmark behind camera')
                continue; % skip
            end
            
            
            % perform normal check
            patchNormInCurrentFrame = T_target_source(1:3, 1:3) * graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.patchNormal;
            
            % Assume patchNorm is unit vec
            assert(abs(norm(patchNormInCurrentFrame) - 1) < 1e-6);
            
            if -1 * patchNormInCurrentFrame(3) < minimumCos
                disp('potential landmark norm not facing cam');
                continue;
            end
            
            % at this point we can call this landmark 'visible'
            try
                [px] = frame.cameraModel.project(r_cam);
            catch e
                fprintf('Project Failed: %s\n', e.message);
                continue;
            end
        
            
            visibleLandmarkIDs{end+1} = {graph.FrameContainer{frameIdx}.ID, graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.ID};
            visibleLandmarkPixelPositions{end+1} = px;
            
        end
    end 
    
    if keyframesChecked >= maximumKeyframes
        break;
    end
end

end

