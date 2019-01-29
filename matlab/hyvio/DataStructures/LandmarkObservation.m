classdef LandmarkObservation < handle
    %OBSERVATION a set of potential correspondences between a landmark and
    %image. This is used to compute a GMM correspondence distribution. It
    %is a hybrid between an indirect and direct method.
    %
    % each potential correspondence has an associated score.
    
    properties
        potentialCorrespondenceSet = {} % the set of all potential correspondences for a landmark
        
        observationFrameID % id of observation keyframe in pose graph.
        landmarkParentFrameID % id of landmark keyframe in pose graph.
        landmarkID % id of landmark in keyframe.
        
    end
    
    methods
        
    end
end

