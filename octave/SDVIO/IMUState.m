classdef IMUState
    % Stores the state and corresponding uncertainty of the IMU. This
    % includes the IMU biases. This is updated at imu rate.
    
    properties
        r % position in world.
        q % attitude in world.
        v % velocity in world.
        
        biases % IMU biases [acc, gyro]
        
        Sigma % IMU state uncertainty. 15x15 covariance matrix.
    end
    
    methods
        
    end
end

