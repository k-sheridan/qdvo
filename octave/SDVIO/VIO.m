classdef VIO
    %VIO The main VIO implementation.
    
    properties
        graph % pose graph / map
        
    end
    
    methods
        function obj = VIO(T_camFromImu, accelBias, gyroBias, accelScale, gyroScale)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
        end
        
        function obj = addFrame(image, t, focal, principal, distortion, vignette)
            % Add an image to the VIO pipeline. distortion params are from
            % equidistant distortion model.
        end
        
        function obj = addIMUSample(accel, gyro, t)
            % Add an imu sample to the VIO pipeline.
        end
    end
end

