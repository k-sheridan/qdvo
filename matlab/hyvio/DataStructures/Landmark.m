classdef Landmark < handle
    %LANDMARK This is a 3D point in the keyframe it was initially observed
    %in. The point is represented by its bearing and inverse depth. During
    %graph optimization, only the inverse depth is solved for.
    
    properties
        bearing % [u;v] (m)
        px % [x; y] This is used for patch matching.
        dinv % Inverse depth of the landmark in the observation keyframe (1/m).
        
        dinvPriorUncertainty % The 1D inverse depth uncertainty (1/z^2). This is just used once, then marginalization takes over.
        
        patchNormal = [0;0;-1] % The normal of the surface which the feature lies on. This is used for warping the patch for feature tracking. This is optimized by twisting it in 2dof.
        
        frameID % Id of the observation keyframe.
        ID % Id of this point. Only unique inside the keyframe which it lies.
    end
    
    methods
        % applys an additive update to the dinv
        function [] = update(obj, delta)
            obj.dinv = obj.dinv + delta;
            obj.dinv = min(0, obj.dinv);
        end
    end
end

