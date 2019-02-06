classdef InertialErrorTerm < handle
    %INERTIALCONSTRAINT stores the information necessary to form both a
    %preintegrated and full inertial constraint series
    
    properties
        parentFrameID = -1;
        childFrameID = -1;
        
        imuMeasurementArray = {}; % cell array of IMUMeasurement
        
        initialized = false; % has the preintegrated measurement been computed?
        
        % These are the preintegrated measurements with information.
        deltaRotation = eye(3); % relative rotation between parent and child \in SO(3) (rotation matrix)
        deltaVelocity = zeros(3, 1); % change in velocity from parent to child
        deltaPosition = zeros(3, 1); % change in position from parent to child
        
        % biases do not change!
        
        % order of variables: [drot (so(3)), dvel, dpos, dbias_g, dbias_a]
        biasJacobian; % jacobian used to make the preintegrated deltas a linear function of the biases. These must be recomputed if the bias delta is too large (TBD).
        
        % covariance matrix
        P; % 15X15 covariance matrix representing the uncertainty of these deltas.
        
        
    end
    
    methods
        
        % iteratively integrate the imu measurements with the bias estimate
        % given. Further, propagate the noise into a covariance matrix.
        % This is all based off cfo's on manifold preintegration paper.
        function [] = preintegrateIMUMeasurements(obj, biases)
            disp('not integrating IMU')
        end
        
    end
end

