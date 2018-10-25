classdef IMUState
    % Stores the state and corresponding uncertainty of the IMU. This
    % includes the IMU biases. This is updated at imu rate.
    
    properties
        r % position in world.
        q % attitude in world. (Quaternion: [qx, qy, qz, qw])
        v % velocity in world.
        
        biases % IMU biases [acc, gyro]
        
        Sigma % IMU state uncertainty. 15x15 covariance matrix.
    end
    
    methods
        function obj = IMUState()
            % default constructor.
            r = zeros(3, 1);
            q = [0;0;0;1];
            v = zeros(3, 1);
            biases = zeros(6, 1);
            
            Sigma = zeros(15);
        end
    end
end

