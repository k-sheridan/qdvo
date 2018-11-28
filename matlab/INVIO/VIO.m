classdef VIO
    %VIO The main VIO implementation.
    % This is not designed to run fast. It is designed to be easily
    % modified, and test new marginalization, feature tracking, feature
    % selection methods, etc.
    
    properties
        graph % pose graph / map
       
        frameBuffer = {} % stores last N frames
        imuBuffer = {} % stores last N imu measurements
        
        settings; % settings for the whole vio impl
    end
    
    methods
        function obj = VIO(settings)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
            obj.settings = settings;
        end
        
        function [obj] = addFrame(obj, frame)
            % Add an image to the VIO pipeline. distortion params are from
            % equidistant distortion model.
            obj.frameBuffer{end+1} = frame;
            if (length(obj.frameBuffer) > obj.settings.maxBufferedFrames)
                obj.frameBuffer = obj.frameBuffer(2:end);
            end
            
            % Create a frame object using last frame and buffered IMU
            % readings.
            
            % Track Landmarks from the last N keyframes in the graph
            
            % Run frontend visual inertial pose optimization
            
            % Check if current frame is a keyframe.
            if (isKeyFrame(obj.frameBuffer{end}, obj.graph))
                % Add a keyframe to the graph.
                
                % Take the IMU measurements and store them in an inertial
                % constraint.
                
                
                % Add the inertial constraint to the graph
                
            end
            
        end
        
        function [obj] = addIMUMeasurement(obj, imuMeasurement)
            % Add an imu sample to the VIO pipeline.
            
        end
        
    end
end

