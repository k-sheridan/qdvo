classdef VIO
    %VIO The main VIO implementation.
    
    properties
        
    end
    
    methods
        function obj = VIO(inputArg1,inputArg2)
            %VIO Construct a VIO instance
            %obj.Property1 = inputArg1 + inputArg2;
        end
        
        function obj = addFrame(image, t, focal, principal, distortion, vignette)
            % Add an image to the VIO pipeline.
        end
        
        function obj = addIMUSample(accel, gyro, t)
            % Add an imu sample to the VIO pipeline.
        end
    end
end

