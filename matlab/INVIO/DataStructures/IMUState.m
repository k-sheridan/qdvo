classdef IMUState
    % Stores the state and corresponding uncertainty of the IMU. This
    % includes the IMU biases. This is updated at imu rate.
    
    properties
        r % position in world.
        q % attitude in world. (Quaternion: [qw, qx, qy, qz])
        v % velocity in world.
        w % angular velocity body frame
        
        biases % IMU biases [acc, gyro]
        
        Sigma % IMU state uncertainty. 15x15 covariance matrix.
    end
    
    methods
        function obj = IMUState()
            % default constructor.
            % \
            obj.r = zeros(3, 1); % position of imu (inertial frame)
            obj.q = [1;0;0;0]; % quaternion of imu (inertial frame)
            obj.v = zeros(3, 1); % velocity of imu (inertial frame)
            obj.w = zeros(3, 1); % angular velocity of imu (body frame) (for more generic motion models)
            obj.biases = zeros(6, 1);
            
            obj.Sigma = zeros(18);
        end
        
        function [T] = poseTransform(obj)
            % create a 4x4 transformation matrix from this state
            T = [[quat2rotm(obj.q'); zeros(1, 3)], [obj.r; 1]];
        end
        
    end
end

