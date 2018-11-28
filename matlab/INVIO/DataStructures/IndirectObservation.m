classdef IndirectObservation
    %OBSERVATION a pixel position observation of a landmark.
    % includes a residual function and corresponding jacobians.
    
    properties
        px % pixel observation of the landmark
        uv % undistorted bearing observation of the landmark
        unprojectJacobian % 2x2 jacobian which maps pixel error into the image plane.
        pixelUncertainty % 2x2 covariance matrix in pixels representing the uncertainty in the feature match. This is used to support edgelet features.
        
        observationKfId % id of observation keyframe in pose graph.
        landmarkKfId % id of landmark keyframe in pose graph.
        landmarkId % id of landmark in keyframe.
        
    end
    
    methods
        
        % this function computes the landmark reprojection residual (2x1),
        % and optional jacobians. inpute types: (IMUState, IMUState, Landmark)
        function [chi, J_observingCameraPose, J_landmarkDepthInverse, J_landmarkCameraPose] = residual(obj, observingFrame, landmarkFrame, landmark)
            % LM_world = T_lmFromWorld * LM_lm
            % LM_obs = inv(T_obsFromWorld) * LM_world
            %  = inv(T_obsFromWorld) * T_lmFromWorld * LM_lm
            % = inv(T_obsFromWorld) * T_lmFromWorld * [u;v;1] * 1/zinv
            % PI(x) = [x(1)/x(3); x(2)/x(3)]
            % uv_obs = PI(inv(T_obsFromWorld) * (T_lmFromWorld * [u;v;1] * 1/zinv))
            % uv_obs = PI(inv(T_obsFromWorld) * [R_lmFromWorld * ([u;v;1] * 1/zinv) + t_lmFromWorld])
            % uv_obs = PI(R_obsFromWorld' * [R_lmFromWorld * ([u;v;1] * zinv^(-1)) + t_lmFromWorld] - R_obsFromWorld' * t_obsFromWorld)
        end
        
    end
end

