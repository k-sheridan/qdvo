classdef KeyFrame
    %KEYFRAME used inside the pose graph.
    
    properties
        frame %FRAME Stores image info and intrinsic parameters.
        state %IMUSTATE Stores the state of this keyframe.
        id %INT unique keyframe id. This id is used to associate landmarks and measurements with keyframe.
        
        landmarks %In this method landmarks are represented in the frame from which they were first observed.
    end
    
    methods
        
    end
end