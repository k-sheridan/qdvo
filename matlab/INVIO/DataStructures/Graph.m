classdef Graph
    %GRAPH This is the pose graph which is constantly optimized. All
    % slam information is stored here.
    
    properties
        FrameContainer = {} % Cell array of Frames. 
        LandmarkObservationContainer = {} % Cell array of landmark bearing measurements
        InertialErrorTermContainer = {} % Cell array of inertial motion constraints
    end
    
    methods
        
    end
end

