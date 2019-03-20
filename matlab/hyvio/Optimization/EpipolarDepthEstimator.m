classdef EpipolarDepthEstimator < handle
    %DEPTHESTIMATOR looks into the frame history to generate a
    %set of 1D correspondence distributions. Additionally serves as a
    %premptive outlier rejection method similar to the depth filter in SVO.
    
    
    properties
        
    end
    
    methods
        function obj = EpipolarDepthEstimator()
        end
        
        % Will do a linear search along the epipolar line for the landmark
        % by finding it's depth.
        % returns an array of [[dinv;score], [dinv;score], ...]
        % score \in [0,1]
        function [dinvScoreArray] = linearDepthSearch(obj, graph, parentFrameID, landmarkID, targetFrameID, minimumDepth, maximumDepth, resolution)
            dinvScoreArray = [linspace(1/maximumDepth, 1/minimumDepth, resolution); zeros(1,resolution)];
            
            s = Settings();
            
            targetFrame = graph.getFrame(targetFrameID);
            sourceFrame = graph.getFrame(parentFrameID);
            landmark = sourceFrame.landmarks{sourceFrame.getLandmarkIndex(landmarkID)};
            
            % warp a template into the target frame.
            [warpedPatch] = warpPatchToTargetFrame(landmark, ...
                sourceFrame, targetFrame, graph, s.patchHalfSize);
            
            T_i_c = graph.extrinsics.getImu2CameraTransform(targetFrame.camID);
            T_w_sourcei = sourceFrame.imustate.poseTransform();
            T_w_targeti = sourceFrame.imustate.poseTransform();
            
            T_target_source = inv(T_w_targeti * T_i_c) * T_w_sourcei * T_i_c;
            
            pm = ZNCCPatchMatcher(s);
            
            for idx = (1:resolution)
                dinv = dinvScoreArray(1, idx);
                
                p_obs = T_target_source(1:3, 1:4) * [[landmark.bearing; 1] * (1/dinv); 1];
                
                px_obs = targetFrame.cameraModel.project(p_obs);
                
                % extract a patch to compare.
                targetPatch = subPixelPatchFromImage(targetFrame.raw_image, px_obs, s.patchHalfSize);
                
                score = pm.zncc(warpedPatch, targetPatch);
                
                dinvScoreArray(2, idx) = score;
                
            end
            
        end
        
    end
end

