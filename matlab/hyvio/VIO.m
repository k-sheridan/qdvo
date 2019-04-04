classdef VIO < handle
    %VIO The main VIO implementation.
    % This is not designed to run fast. It is designed to be easily
    % modified, and test new marginalization, feature tracking, feature
    % selection methods, etc.
    
    properties
        graph % pose graph / map
        interimPreintegrationTerm = PreintegratedIMUMeasurement(); % used to cache the set of IMU's between frames.
        swe; % a global sliding window estimator
        settings; % settings for the whole vio impl
    end
    
    methods
        function obj = VIO(settings)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
            obj.settings = settings;
            obj.interimPreintegrationTerm = PreintegratedIMUMeasurement();
            obj.graph.extrinsics.addIMU2CameraExtrinsic(1, obj.settings.initial_T_camFromImu(1:3, 1:3), obj.settings.initial_T_camFromImu(1:3, 4));
            
            % create a sliding window estimator.
            obj.swe = SlidingWindowEstimator(obj.settings.windowSize);
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
                
                % The newest keyframe is always active.
                frame.status = FrameStatus.ACTIVE;
                
                % create new landmarks
                frame = createNewLandmarks(frame, obj.graph, obj.settings.nFeaturesDesired);
                
                % initialize the imu biases
                frame.imustate.biases = [obj.settings.initial_accelBias; obj.settings.initial_gyroBias];
                
                % Add the frame to the graph
                obj.graph.addFrame(frame);
                
                % activate landmarks
                obj.graph = activateNewLandmarks(obj.graph);
                
                % add empty frame observations
                fo = FrameObservationContainer();
                fo.frameID = frame.ID;
                obj.graph.FrameObservationContainer{end+1} = fo;
                
                % check if we need to activate new landmarks.
                [idArr] = computeVisibleLandmarks(obj.graph.FrameContainer{end}, obj.graph, true, true);
                if length(idArr) < obj.settings.minimumActiveLandmarks
                    fprintf('Activating new landmarks\n');
                    obj.graph = activateNewLandmarks(obj.graph);
                end
                
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
            frame.imustate.R = obj.graph.FrameContainer{end}.imustate.R * obj.interimPreintegrationTerm.deltaR
            %frame.imustate.R = obj.graph.FrameContainer{end}.imustate.R;
            
            % add the frame to the graph
            obj.graph.addFrame(frame);
            
            % add this inerital error term to the graph as an edge between the frame.
            obj.graph.addInertialConstrain(obj.interimPreintegrationTerm);
            
            
            % compute correspondence models for the active landmarks visible in
            % this frame.
            [landmarkObservations] = computeCorrespondenceModels(obj.graph.FrameContainer{end}, obj.graph);
            fprintf('Found %i correspondence models for active features\n', length(landmarkObservations));
            
            % Add the observations to the graph
            frameObs = FrameObservationContainer();
            frameObs.frameID = frame.ID;
            frameObs.landmarkObservations = landmarkObservations;
            obj.graph.FrameObservationContainer{end+1} = frameObs;
            
            % reset the interim inertial constraint.
            obj.interimPreintegrationTerm = PreintegratedIMUMeasurement();
            
            % front end visual odometry
            obj.runFrontEndVisualOdometry(obj.graph.FrameContainer{end}.ID);
            
            % check if the current frame is a keyframe
            if isKeyframe(obj.graph, obj.graph.FrameContainer{end}.ID)
                fprintf('Creating new landmarks in new keyframe.\n');
                obj.graph.FrameContainer{end} = createNewLandmarks(obj.graph.FrameContainer{end}, obj.graph, obj.settings.nFeaturesDesired);
                obj.graph.FrameContainer{end}.status = FrameStatus.ACTIVE;
                fprintf('Found %i new landmarks\n', length(obj.graph.FrameContainer{end}.landmarks));
                
                % run the sliding window estimator
                obj.runSlidingWindowEstimator();
                
                % remove outlier landmarks after the full optimization
                obj.graph = obj.swe.removeOutliers(obj.graph);
                
                % run the epipolar depth estimator updates
                obj.runEpipolarDepthEstimators();
                
                % check if we need to activate new landmarks.
                [idArr] = computeVisibleLandmarks(obj.graph.FrameContainer{end}, obj.graph, true, true);
                if length(idArr) < obj.settings.nActiveLandmarks
                    fprintf('Activating new landmarks\n');
                    obj.graph = activateNewLandmarks(obj.graph);
                end
                
            end
            
        end
        
        
        function [] = addIMUMeasurement(obj, imuMeasurement)
            %add this measurement to the temporary IMU preintegration.
            if ~isempty(obj.graph.FrameContainer)
                disp('Add IMU measurement to temp InertialErrorTerm')
                obj.interimPreintegrationTerm.imuMeasurementArray{end+1} = imuMeasurement;
            else
                %disp('No frame has been added yet, skipping IMU measurement.')
            end
        end
        
        
        % This will set up and run the sliding window estimator given the
        % current state of the system. It will also marginalize out old
        % states locally.
        function [] = runSlidingWindowEstimator(obj)
            obj.swe.initializeVisionOnly(obj.graph);
            obj.graph = obj.swe.optimize(obj.graph);
            % marginalize if necessaary
            obj.graph = obj.swe.runMarginalizationStrategy(obj.graph);
        end
        
        function [] = runVisualBundleAdjustment(obj)
            vba = VisualBA();
            vba.initialize(obj.graph);
            obj.graph = vba.optimize(obj.graph);
        end
        
        
        % This function will attempt to find the, roughly, the camera
        % pose at the current frame using the active set of landmarks.
        function [] = runFrontEndVisualOdometry(obj, frameID)
            o = Optimizer();
            
            idx = obj.graph.getFrameObservationsIndex(frameID);
            
            for lo = obj.graph.FrameObservationContainer{idx}.landmarkObservations
                et = QuasiDirectErrorTerm_obsFrame(lo{1});
                o.addErrorTerm(et);
            end
            
            o.initialize(obj.graph);
            
            obj.graph = o.optimize(obj.graph);
        end
        
        % This function runs the epipolar depth estimators for inactive
        % landmarks hosted in active frames. updates the n landmarks which
        % were updated longest ago;
        function [] = runEpipolarDepthEstimators(obj)
            
            for idx = (1:length(obj.graph.FrameContainer))
                if obj.graph.FrameContainer{idx}.status == FrameStatus.ACTIVE && idx ~= length(obj.graph.FrameContainer)
                    assert(obj.graph.FrameContainer{idx}.isKeyframe);
                    % sweep all landmarks
                    for lidx = (1:length(obj.graph.FrameContainer{idx}.landmarks))
                        if obj.graph.FrameContainer{idx}.landmarks{lidx}.status == LandmarkStatus.INACTIVE
                            if ~obj.graph.FrameContainer{idx}.landmarks{lidx}.epipolarDepthEstimator.initialized
                                % update the epipolar depth estimator
                                obj.graph = obj.graph.FrameContainer{idx}.landmarks{lidx}.epipolarDepthEstimator.updateLandmark(obj.graph);
                            end
                        end
                    end
                end
            end
            
        end
        
    end
end

