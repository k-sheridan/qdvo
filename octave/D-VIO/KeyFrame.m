classdef KeyFrame
    %KEYFRAME used inside the pose graph.
    
    properties
        frame %FRAME Stores image info and intrinsic parameters.
        state %POSESTATE Stores the state of this keyframe.
        id %INT unique keyframe id. This id is used to associate landmarks and measurements with keyframe.
    end
    
    methods
        
    end
end