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
        
        %Assume biases do not change!
        biasLinearlizationPoint = []; % the bias vector at which the term was preintegrated. [ba;bg]
        
        dDR_dbg = []; %3X3
        dDV_dba = []; %3X3
        dDV_dbg = []; %3X3
        dDP_dba = []; %3X3
        dDP_dbg = []; %3X3
        
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
            
            obj.dDR_dbg = zeros(3);
            obj.dDV_dbg = zeros(3);
            obj.dDP_dbg = zeros(3);
            obj.dDV_dba = zeros(3);
            obj.dDP_dba = zeros(3);
            
            obj.biasLinearlizationPoint = biases;
            
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
                    zeros(3), rightJacobianOfSO3(obj.imuMeasurementArray{idx}.gyro - biasGyro) * dti
                    obj.deltaR * dti, zeros(3);];
                
                % propagate the uncertainty
                noise = obj.imuMeasurementArray{idx}.getCov();
                
                obj.P(1:9, 1:9) = A * obj.P(1:9, 1:9) * A' + B * noise(1:6, 1:6) * B';
                
                % add bias walk
                obj.P(10:15, 10:15) = noise(7:12, 7:12) * dti^2;
                
                % compute integrate delta
                obj.deltaP = obj.deltaP + obj.deltaV * dti + 1/2 * obj.deltaR * (obj.imuMeasurementArray{idx}.accel - biasAccel) * dti^2;
                obj.deltaV = obj.deltaV + obj.deltaR * (obj.imuMeasurementArray{idx}.accel - biasAccel) * dti;
                obj.deltaR = obj.deltaR * dRi; % gives dR_i_k+1
                
                % Compute bias jacobians.
                % first handle the rotation jacobian
                obj.dDR_dbg = obj.dDR_dbg + obj.deltaR * rightJacobianOfSO3((obj.imuMeasurementArray{idx}.gyro - biasGyro)) * dti;
                obj.dDV_dba = obj.dDV_dba + dti;
                obj.dDV_dbg = obj.dDV_dbg + so3Hat(obj.imuMeasurementArray{idx}.accel - biasAccel) * dti;
                obj.dDP_dba = obj.dDP_dba + dti^2;
                obj.dDP_dbg = obj.dDP_dbg + so3Hat(obj.imuMeasurementArray{idx}.accel - biasAccel) * dti^2;
                
                
                obj.dt = obj.dt + dti;
            end
            
            % Finish the bias jacobians
            obj.dDR_dbg = -obj.deltaR' * obj.dDR_dbg;
            obj.dDV_dba = -obj.deltaR * obj.dDV_dba;
            obj.dDV_dbg = -obj.deltaR * obj.dDV_dbg * obj.dDR_dbg;
            obj.dDP_dba = -3/2 * obj.deltaR * obj.dDP_dba;
            obj.dDP_dbg = -3/2 * obj.deltaR * obj.dDP_dbg * obj.dDR_dbg;
            
            obj.initialized = true;
            
        end
        
        
    end
end

