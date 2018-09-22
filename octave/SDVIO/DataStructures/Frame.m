classdef Frame
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (mono8).
        time % Capture time in seconds
        
        cameraCalibration
        
        state %IMUSTATE Stores the state of this keyframe.
        
    end
    
    methods
        function obj = Frame(image, t, f, c, d, imuState)
            % constructs a frame object.
            obj.raw_image = image;
            obj.time = t;
            obj.cameraCalibration.principalPoint = c;
            obj.cameraCalibration.focalLength = f;
            obj.cameraCalibration.distortionCoefficients = d;
            obj.cameraCalibration.imageSize = circshift(size(image), 1);
            obj.state = imuState;
        end
    end
end

