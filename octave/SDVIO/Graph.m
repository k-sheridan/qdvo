classdef Graph
    %GRAPH This is the pose graph which is constantly optimized. All
    % slam information is stored here.
    
    properties
        keyframeContainer % Array of KeyFrames. 
        observationContainer % Array of landmark bearing measurements
        
        %TODO add interkeyframe integrated imu motion constraints. keep all
        %raw measurements
    end
    
    methods
        
    end
end

