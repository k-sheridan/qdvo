classdef EpipolarDepthEstimator < handle
    %DEPTHESTIMATOR looks into the frame history to generate a
    %set of 1D correspondence distributions. Additionally serves as a
    %premptive outlier rejection method similar to the depth filter in SVO.
    
    
    properties 
        parentFrameID;
        landmarkID;
        updateHistory = {}; % {{updateFrameID, [dinvScoreArray]}, {updateFrameID, [dinvScoreArray]}, ....}
    end
    
    methods
        function obj = EpipolarDepthEstimator(parentFrameID, landmarkID)
            obj.parentFrameID = parentFrameID;
            obj.landmarkID = landmarkID;
            obj.updateHistory = {};
        end
        
        
        % This function will both update the landmark dinv and update its
        % variance.
        function [graph] = updateLandmark(obj, graph)
            s = Settings();
            
            [dinvScoreArray] = obj.linearDepthSearch(graph, obj.parentFrameID, obj.landmarkID, graph.FrameContainer{end}.ID,...
                s.minimumDepth, s.maximumDepth, s.resolution);
            
            obj.updateHistory{end+1} = {graph.FrameContainer{end}.ID, dinvScoreArray};
            
            fidx = graph.getFrameIndex(obj.parentFrameID);
            lidx = graph.FrameContainer{fidx}.getLandmarkIndex(obj.landmarkID);
            
            % update the landmark depth.
            [maximum, index] = max(dinvScoreArray(2, :));
            dinvMax = dinvScoreArray(1, index);
            graph.FrameContainer{fidx}.landmarks{lidx}.dinv = dinvMax;
            
            % compute the variance
            n = 0;
            sumSquaredDiff = 0;
            for index = (1:length(dinvScoreArray(2, :)))
                if dinvScoreArray(2, index) >= s.minimumNormalizedMatchCorrelation
                    sumSquaredDiff = sumSquaredDiff + (dinvScoreArray(1, index) - dinvMax)^2;
                    n = n + 1;
                end
            end
            variance = sumSquaredDiff / n;
            graph.FrameContainer{fidx}.landmarks{lidx}.dinvPriorUncertainty = variance;
            
            %TODO check if this is an outlier.
            
        end
        
        % Will do a linear search along the epipolar line for the landmark
        % by finding it's depth.
        % returns an array of [[dinv;score], [dinv;score], ...]
        % score \in [0,1]
        function [dinvScoreArray] = linearDepthSearch(obj, graph, parentFrameID, landmarkID, targetFrameID, minimumDepth, maximumDepth, resolution)
            dinvScoreArray = [linspace(minimumDepth, maximumDepth, resolution).^(-1); zeros(1,resolution)];
            
            s = Settings();
            
            targetFrame = graph.getFrame(targetFrameID);
            sourceFrame = graph.getFrame(parentFrameID);
            landmark = sourceFrame.landmarks{sourceFrame.getLandmarkIndex(landmarkID)};
            
            % warp a template into the target frame.
            [warpedPatch] = warpPatchToTargetFrame(landmark, ...
                sourceFrame, targetFrame, graph, s.patchHalfSize);
            
            T_i_c = graph.extrinsics.getImu2CameraTransform(targetFrame.camID);
            T_w_sourcei = sourceFrame.imustate.poseTransform();
            T_w_targeti = targetFrame.imustate.poseTransform();
            
            T_target_source = inv(T_w_targeti * T_i_c) * T_w_sourcei * T_i_c;
            
            pm = ZNCCPatchMatcher(s);
            
            for idx = (1:resolution)
                dinv = dinvScoreArray(1, idx);
                
                p_obs = T_target_source(1:3, 1:4) * [[landmark.bearing; 1] * (1/dinv); 1];
                
                px_obs = targetFrame.cameraModel.project(p_obs);
                
                % extract a patch to compare.
                targetPatch = subPixelPatchFromImage(targetFrame.raw_image, px_obs, s.patchHalfSize);
                %imagesc(targetPatch.image, [0, 2^16]);
                %drawnow;
                
                score = pm.zncc(warpedPatch, targetPatch);
                
                dinvScoreArray(2, idx) = score;
                
            end
            
        end
        
    end
end

