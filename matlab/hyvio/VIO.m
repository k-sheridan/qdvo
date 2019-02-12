classdef VIO < handle
    %VIO The main VIO implementation.
    % This is not designed to run fast. It is designed to be easily
    % modified, and test new marginalization, feature tracking, feature
    % selection methods, etc.
    
    properties
        graph % pose graph / map
        interimPreintegrationTerm = PreintegratedIMUMeasurement(); % used to cache the set of IMU's between frames.
        settings; % settings for the whole vio impl
    end
    
    methods
        function obj = VIO(settings)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
            obj.settings = settings;
            obj.interimPreintegrationTerm = PreintegratedIMUMeasurement();
            obj.graph.extrinsics.addIMU2CameraExtrinsic(1, obj.settings.initial_T_camFromImu(1:3, 1:3), obj.settings.initial_T_camFromImu(1:3, 4));
        end
        
        function [] = addFrame(obj, frame)
            % Handle the first frame
            if isempty(obj.graph.FrameContainer)
                frame.ID = 1;
                
                % prep interim Inertial error term
                obj.interimPreintegrationTerm = PreintegratedIMUMeasurement();
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
            
            % pre integrate the temp inertial error term to initialize the
            % imu state of this frame.
            obj.interimPreintegrationTerm.preintegrateIMUMeasurements(obj.graph.FrameContainer{end}.imustate.biases, obj.graph.FrameContainer{end}.t, frame.t);
            obj.interimPreintegrationTerm.parentFrameID = obj.graph.FrameContainer{end}.ID;
            obj.interimPreintegrationTerm.childFrameID = frame.ID;
            
            % initialize this frame's imustate
            frame.imustate.biases = obj.graph.FrameContainer{end}.imustate.biases;
            %TODO initialize the position and velocity with inertial error
            %term
            frame.imustate.p = obj.graph.FrameContainer{end}.imustate.p;
            frame.imustate.v = obj.graph.FrameContainer{end}.imustate.v;
            % always use gyro to initialize the orientation.
            frame.imustate.R = obj.graph.FrameContainer{end}.imustate.R * obj.interimPreintegrationTerm.deltaR;
            
            % add the frame to the graph
            obj.graph.addFrame(frame);
            
            % add this inerital error term to the graph as an edge between the frame.
            obj.graph.addInertialConstrain(obj.interimPreintegrationTerm);
            
            % compute correspondence models for the landmarks visible in
            % this frame.
            [landmarkObservations] = computeCorrespondenceModels(obj.graph.FrameContainer{end}, obj.graph);
            
            % Add the observations to the graph
            frameObs = FrameObservationContainer();
            frameObs.frameID = frame.ID;
            frameObs.landmarkObservations = landmarkObservations;
            obj.graph.LandmarkObservationContainer{end+1} = frameObs;
            
            % reset the interim inertial constraint.
            obj.interimPreintegrationTerm = PreintegratedIMUMeasurement();
            
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

