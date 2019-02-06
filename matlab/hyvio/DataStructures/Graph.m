classdef Graph < handle
    %GRAPH This is the pose graph which is constantly optimized. All
    % slam information is stored here.
    
    properties
        FrameContainer = {} % Cell array of Frames. 
        LandmarkObservationContainer = {} % Cell array of landmark bearing measurements, ({{observations in frame 1}, {observations in frame 2}, etc.})
        InertialConstraintContainer = {} % Cell array of inertial motion constraints
        extrinsics = Extrinsics();
    end
    
    methods
        
        function [] = addFrame(obj, frame)
            obj.FrameContainer{end+1} = frame;
        end
        
        function [] = addInertialConstrain(obj, inertialErrorTerm)
            if ~inertialErrorTerm.initialized
                disp('Inertial Constraint not preintegrated');
            end
            obj.InertialConstraintContainer{end+1} = inertialErrorTerm;
        end
        
    end
end

