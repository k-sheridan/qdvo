function [visibleLandmarkIDs, visibleLandmarkPixelPositions] = computeVisibleLandmarks(frame, graph, activeKFsOnly, activelandmarksOnly)
%This functions computes the landmarks visible in a frame given the graph.
% visibleLandmarkIDs: vector of parent + landmark ids: {{parentFrameID, landmarkID}}
% visibleLandmarkPixelpositions: {[x1;y1], [x2;y2], ....}

assert(frame.ID > 0);

if nargin < 3
    activeKFsOnly = false;
end

if nargin < 4
    activelandmarksOnly = false;
end

%TODO check these guys
minimumDepth = 0.01;
minimumCos = cosd(85);
maximumRadiusRatio = 0.8;

visibleLandmarkIDs = {};
visibleLandmarkPixelPositions = {};

T_i_c = graph.extrinsics.getImu2CameraTransform(frame.camID);

for frameIdx = (1:length(graph.FrameContainer))
    if graph.FrameContainer{frameIdx}.isKeyframe
        
        
        if activeKFsOnly && graph.FrameContainer{frameIdx}.status ~= FrameStatus.ACTIVE
            continue;
        end
        
        T_target_source = (frame.imustate.poseTransform() * T_i_c) \ ...
                (graph.FrameContainer{frameIdx}.imustate.poseTransform() * T_i_c);
        
        for landmarkIdx = (1:length(graph.FrameContainer{frameIdx}.landmarks))
            % transform and project landmark into the frame given
            assert(graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.frameID == graph.FrameContainer{frameIdx}.ID);
            assert(graph.FrameContainer{frameIdx}.ID > 0);
            
            if activelandmarksOnly && graph.FrameContainer{frameIdx}.landmarks{landmarkIdx}.status ~= LandmarkStatus.ACTIVE
                continue;
            end
            
            % inv(T_w2c)*T_w2l*bearing/dinv
            % best estimate of landmark position in current frame
            
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
    
    
end

end

