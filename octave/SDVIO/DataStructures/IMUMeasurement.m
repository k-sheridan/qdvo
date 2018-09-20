classdef IMUMeasurement
    %IMUMEASUREMENT stores an imu measurement with noises.
    
    properties
        accel % m/s^2
        gyro % rad/s
        
        t % s
        
        % Noises are std dviations
        accelRW % m/s^2.5
        gyroRW % rad/s^1.5
        
        accelNoise % m/s^1.5
        gyroNoise % rad/s^0.5
    end
    
    methods
        
    end
end

