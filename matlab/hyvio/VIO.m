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
            obj.interimPreintegrationTerm = InertialErrorTerm();
            obj.graph.extrinsics.addIMU2CameraExtrinsic(1, obj.settings.initial_T_camFromImu(1:3, 1:3), obj.settings.initial_T_camFromImu(1:3, 4));
        end
        
        function [] = addFrame(obj, frame)
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
                
                % initialize the imu biases
                frame.imustate.biases = [obj.settings.initial_accelBias; obj.settings.initial_gyroBias];
                
                % Add the frame to the graph
                obj.graph.addFrame(frame);
                
                return
            end
            
            frame.ID = obj.graph.FrameContainer{end}.ID + 1; % assign ID to new frame
            
            
            
        end
        
        function [] = addIMUMeasurement(obj, imuMeasurement)
            %add this measurement to the temporary IMU preintegration.
            if ~isempty(obj.graph.FrameContainer)
                disp('Add IMU measurement to temp InertialErrorTerm')
                obj.interimPreintegrationTerm.imuMeasurementArray{end+1} = imuMeasurement;
            else
                disp('No frame has been added yet, skipping IMU measurement.')
            end
        end
        
    end
end

