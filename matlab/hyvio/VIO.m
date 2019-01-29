classdef VIO < handle
    %VIO The main VIO implementation.
    % This is not designed to run fast. It is designed to be easily
    % modified, and test new marginalization, feature tracking, feature
    % selection methods, etc.
    
    properties
        graph % pose graph / map
        interimPreintegrationTerm = InertialErrorTerm(); % used to cache the set of IMU's between frames.
        settings; % settings for the whole vio impl
    end
    
    methods
        function obj = VIO(settings)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
            obj.settings = settings;
        end
        
        function [obj] = addFrame(obj, frame)
            % Handle the first frame
            if isempty(obj.graph.FrameContainer)
                frame.ID = 1;
                
                % prep interim Inertial error term
                obj.interimPreintegrationTerm = InertialErrorTerm();
                obj.interimPreintegrationTerm.parentFrameID = frame.ID;
                
                % since this is the first frame, it is a keyframe
                frame.isKeyframe = true;
                
                % create new landmarks
                frame = createNewLandmarks(frame, obj.graph, obj.settings.nFeaturesDesired);
                
                % Add the frame to the graph
                obj.graph.addFrame(frame);
                
                return
            end
            
            frame.ID = obj.graph.FrameContainer{end}.ID + 1;
            
            
        end
        
        function [obj] = addIMUMeasurement(obj, imuMeasurement)
            %add this measurement to the temporary IMU preintegration.
            if ~isempty(obj.graph.FrameContainer)
                
            else
                disp('No frame has been added yet, skipping IMU measurement.')
            end
        end
        
    end
end

