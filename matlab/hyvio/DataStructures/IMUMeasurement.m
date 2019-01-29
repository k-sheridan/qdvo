classdef IMUMeasurement < handle
    %IMUMEASUREMENT stores an imu measurement with noises.
    
    properties
        accel % m/s^2
        gyro % rad/s
        
        t % s
        
        % Noises are std deviations (3X1)
        accelRandomWalk % m/s^2.5
        gyroRandomWalk % rad/s^1.5
        
        accelNoise % m/s^1.5
        gyroNoise % rad/s^0.5
    end
    
    methods
        function [R] = getCov(obj)
            R = diag([obj.accelNoise; obj.gyroNoise; obj.accelRW; obj.gyroRW].^2);
        end
    end
end

