classdef VIO
    %VIO The main VIO implementation.
    % This is not designed to run fast. It is designed to be easily
    % modified, and test new marginalization, feature tracking, feature
    % selection methods, etc.
    
    properties
        graph % pose graph / map
       
        frameBuffer % stores last N frames
        imuBuffer % stores last N imu measurements
    end
    
    methods
        function obj = VIO(T_camFromImu, accelBias, gyroBias, accelScale, gyroScale)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
        end
        
        function obj = addFrame(image, t, focal, principal, distortion, vignette)
            % Add an image to the VIO pipeline. distortion params are from
            % equidistant distortion model.
            
            % Create a frame object using last frame and buffered IMU
            % readings.
            
            % Track Landmarks from the last N keyframes in the graph
        end
        
        function obj = addIMUSample(imuMeasurement)
            % Add an imu sample to the VIO pipeline.
        end
        
    end
end

