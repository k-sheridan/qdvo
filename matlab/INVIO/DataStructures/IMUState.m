classdef IMUState
    % Stores the state and corresponding uncertainty of the IMU. This
    % includes the IMU biases. This is updated at imu rate.
    
    properties
        p % position in world.
        R % attitude in world. (SO(3), 3x3)
        v % velocity in world.
        
        biases % IMU biases [acc, gyro]
        
        Sigma % IMU state uncertainty. 15x15 covariance matrix.
    end
    
    methods
        function obj = IMUState()
            % default constructor.
            % \
            obj.p = zeros(3, 1); % position of imu (inertial frame)
            obj.R = eye(3); % Rot of imu (inertial frame)
            obj.v = zeros(3, 1); % velocity of imu (inertial frame)
            obj.biases = zeros(6, 1);
            
            obj.Sigma = zeros(18);
        end
        
        function [T] = poseTransform(obj)
            % create a 4x4 transformation matrix from this state
            T = [[obj.R; zeros(1, 3)], [obj.p; 1]];
        end
        
    end
end

