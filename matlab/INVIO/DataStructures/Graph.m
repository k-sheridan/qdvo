classdef Graph
    %GRAPH This is the pose graph which is constantly optimized. All
    % slam information is stored here.
    
    properties
        keyframeContainer % Cell array of KeyFrames. 
        observationConstrainContainer % Cell array of landmark bearing measurements
        inertialConstraintContainer % Cell array of inertial motion constraints
        
        %TODO add interkeyframe integrated imu motion constraints. keep all
        %raw measurements
    end
    
    methods
        
    end
end

