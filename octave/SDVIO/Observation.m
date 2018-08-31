classdef Observation
    %OBSERVATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        px % pixel observation of the landmark
        uv % undistorted bearing observation of the landmark
        unprojectJacobian % 2x2 jacobian which maps pixel error into the image plane.
        pixelUncertainty % 2x2 covariance matrix in pixels representing the uncertainty in the feature match. This is used to support edgelet features.
        
        kfid % id of keyframe in pose graph.
        pointid % id of landmark in keyframe.
    end
    
    methods
        
    end
end

