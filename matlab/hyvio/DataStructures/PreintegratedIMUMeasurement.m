classdef PreintegratedIMUMeasurement < handle
    %INERTIALCONSTRAINT stores the information necessary to form both a
    %preintegrated and full inertial constraint series
    
    properties
        parentFrameID = -1;
        childFrameID = -1;
        
        dt = -1;
        
        imuMeasurementArray = {}; % cell array of IMUMeasurement
        
        initialized = false; % has the preintegrated measurement been computed?
        
        % These are the preintegrated measurements with information.
        deltaR = eye(3); % relative rotation between parent and child \in SO(3) (rotation matrix)
        deltaV = zeros(3, 1); % change in velocity from parent to child
        deltaP = zeros(3, 1); % change in position from parent to child
        
        % biases do not change!
        
        % order of variables: [dp, dphi, dv, dba, dbg]
        biasJacobian; % jacobian used to make the preintegrated deltas a linear function of the biases. These must be recomputed if the bias delta is too large (TBD).
        
        % covariance matrix
        % order of variables: [dp, dphi, dv, dba, dbg]
        P; % 15X15 covariance matrix representing the uncertainty of these deltas.
        
        
    end
    
    methods
        
        % iteratively integrate the imu measurements with the bias estimate
        % given. Further, propagate the noise into a covariance matrix.
        % This is all based off cfo's on manifold preintegration paper.
        % Bias order: [ba; bg]
        function [] = preintegrateIMUMeasurements(obj, biases, t0, tf)
            biasAccel = biases(1:3);
            biasGyro = biases(4:6);
            
            obj.deltaR = eye(3);
            obj.deltaV = zeros(3, 1);
            obj.deltaP = zeros(3, 1);
            
            obj.P = zeros(15);
            
            finalIdx = length(obj.imuMeasurementArray);
            
            obj.dt = 0;
            
            for idx = 1:finalIdx
                dti = 0;
                if idx == 1
                    % assume that the imu measurment is valid from t0 to
                    % the next imu 
                    dti = obj.imuMeasurementArray{idx+1}.t - t0;
                elseif idx == finalIdx
                    % use tf as the upper bound
                    dti = tf - obj.imuMeasurementArray{idx}.t;  
                else
                    dti = obj.imuMeasurementArray{idx+1}.t - obj.imuMeasurementArray{idx}.t;
                end
                
                dRi = so3Exp((obj.imuMeasurementArray{idx}.gyro - biasGyro) * dti);
                
                % set up error state dynamics
                % order: [dp, dphi, dv, dba, dbg]
                A = [eye(3), -1/2 * obj.deltaR * so3Hat(obj.imuMeasurementArray{idx}.accel - biasAccel) * dti^2, diag([dti;dti;dti]);
                    zeros(3), dRi', zeros(3);
                    zeros(3), -obj.deltaR * so3Hat(obj.imuMeasurementArray{idx}.accel - biasAccel) * dti, eye(3)];
                
                % B 
                B = [1/2 * obj.deltaR * dti^2, zeros(3);
                    zeros(3), rightJacobianOfSO3(so3Log(obj.deltaR)) * dti
                    obj.deltaR * dti, zeros(3);];
                
                % propagate the uncertainty
                noise = obj.imuMeasurementArray{idx}.getCov();
                
                obj.P(1:9, 1:9) = A * obj.P(1:9, 1:9) * A' + B * noise(1:6, 1:6) * B';
                
                % add bias walk
                obj.P(10:15, 10:15) = noise(7:12, 7:12) * dti^2;
                
                % compute integrate delta
                obj.deltaP = obj.deltaP + obj.deltaV * dti + 1/2 * obj.deltaR * (obj.imuMeasurementArray{idx}.accel - biasAccel) * dti^2;
                obj.deltaV = obj.deltaV + obj.deltaR * (obj.imuMeasurementArray{idx}.accel - biasAccel) * dti;
                obj.deltaR = obj.deltaR * dRi;
                
                %TODO compute bias jacobians
                
                
                obj.dt = obj.dt + dti;
            end
            
            obj.initialized = true;
            
        end
        
        
    end
end

