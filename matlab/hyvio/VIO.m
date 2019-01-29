classdef VIO < handle
    %VIO The main VIO implementation.
    % This is not designed to run fast. It is designed to be easily
    % modified, and test new marginalization, feature tracking, feature
    % selection methods, etc.
    
    properties
        graph % pose graph / map
        
        settings; % settings for the whole vio impl
    end
    
    methods
        function obj = VIO(settings)
            %VIO Construct a VIO instance. scales are 3x3 matrices
            obj.graph = Graph();
            obj.settings = settings;
        end
        
        function [obj] = addFrame(obj, frame)
            
        end
        
        function [obj] = addIMUMeasurement(obj, imuMeasurement)
            
        end
        
    end
end

