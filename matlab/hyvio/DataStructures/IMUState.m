classdef IMUState < handle
    % Stores the state and corresponding uncertainty of the IMU. This
    % includes the IMU biases. This is updated at imu rate.
    
    % order of state update vector: [dp, dphi, dv, dba, dbg]
    % dPhi \in so(3). It must be applied to the rotation matrix using
    % exponential map.
    
    properties
        p % position in world.
        R % attitude in world. (SO(3), 3x3)
        v % velocity in world.
        
        biases % IMU biases [acc, gyro]
    end
    
    methods
        function obj = IMUState()
            % default constructor.
            % \
            obj.p = zeros(3, 1); % position of imu (inertial frame)
            obj.R = eye(3); % Rot of imu (inertial frame)
            obj.v = zeros(3, 1); % velocity of imu (inertial frame)
            obj.biases = zeros(6, 1);
        end
        
        function [T] = poseTransform(obj)
            % create a 4x4 transformation matrix from this state
            T = [[obj.R; zeros(1, 3)], [obj.p; 1]];
        end
        
        % run this periodically to correct the numerical error build ups.
        function [] = orthonormalizeRotationMatrix(obj)
            obj.R = quat2rotm(rotm2quat(obj.R));
        end
        
        % applies a small minimal form update to the state. The order is
        % listed above. [dp, dphi, dv, dba, dbg]
        function [] = updateState(obj, dx)
            obj.p = obj.p + dx(1:3);
            obj.R = obj.R * so3Exp(dx(4:6));
            obj.v = obj.v + dx(7:9);
            obj.biases = obj.biases + dx(10:15);
        end
        
    end
end

