classdef LandmarkObservation
    %OBSERVATION a set of potential correspondences between a landmark and
    %image. This is used to compute a GMM correspondence distribution. It
    %is a hybrid between an indirect and direct method.
    %
    % each potential correspondence has an associated score.
    
    properties
        potentialCorrespondenceSet = {} % the set of all potential correspondences for a landmark
        
        observationKfId % id of observation keyframe in pose graph.
        landmarkKfId % id of landmark keyframe in pose graph.
        landmarkId % id of landmark in keyframe.
        
    end
    
    methods
        
    end
end

