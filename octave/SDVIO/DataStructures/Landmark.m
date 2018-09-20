classdef Landmark
    %LANDMARK This is a 3D point in the keyframe it was initially observed
    %in. The point is represented by its bearing and inverse depth. During
    %graph optimization, only the inverse depth is solved for.
    
    properties
        bearing % [u,v] (m)
        px % [x, y] This is used for patch matching.
        zinv % Inverse depth of the landmark in the observation keyframe (1/m).
        
        zinvVariance % The 1D inverse depth uncertainty (1/z^2).
        
        patchNormal % The normal of the surface which the feature lies on. This is used for warping the patch for feature tracking.
        
        kfid % Id of the observation keyframe.
        id % Id of this point. Only unique inside the keyframe which it lies.
    end
    
    methods
        
    end
end

