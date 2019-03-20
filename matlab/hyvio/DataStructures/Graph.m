classdef Graph < handle
    %GRAPH This is the pose graph which is constantly optimized. All
    % slam information is stored here.
    
    properties
        FrameContainer = {} % Cell array of Frames. 
        FrameObservationContainer = {} % Cell array of landmark bearing measurements, ({{observations in frame 1}, {observations in frame 2}, etc.})
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
        
        function [frameIndex] = getFrameIndex(obj, frameID)
            frameIndex = frameID - obj.FrameContainer{1}.ID + 1;
            
            % is this correct?
            if obj.FrameContainer{frameIndex}.ID ~= frameID
                disp('performing linear search to find frame index');
                frameIndex = -1; % this tells us if search failed
                for idx = (1:length(obj.FrameContainer))
                    if obj.FrameContainer{idx}.ID == frameID
                        frameIndex = idx;
                        break;
                    end
                end
                error('index not found for id')
            end
        end
        
        function [f] = getFrame(obj, frameID)
            fidx = obj.getFrameIndex(frameID);
            f = obj.FrameContainer{fidx};
        end
        
        % gets a copy of the landmark by its id.
        function [l] = getLandmark(obj, parentFrameID, landmarkID)
            fidx = obj.getFrameIndex(parentFrameID);
            lidx = obj.FrameContainer{fidx}.getLandmarkIndex(landmarkID);
            l = obj.FrameContainer{fidx}.landmarks{lidx};
        end
        
        function [frameObservationsIndex] = getFrameObservationsIndex(obj, frameID)
            frameObservationsIndex = frameID - obj.FrameObservationContainer{1}.frameID + 1;
            
            % is this correct?
            if obj.FrameObservationContainer{frameObservationsIndex}.frameID ~= frameID
                disp('performing linear search to find frame index');
                frameObservationsIndex = -1; % this tells us if search failed
                for idx = (1:length(obj.FrameObservationContainer))
                    if obj.FrameObservationContainer{idx}.frameID == frameID
                        frameObservationsIndex = idx;
                        break;
                    end
                end
            end
        end
        
    end
end

