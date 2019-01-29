classdef Frame < handle
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (grayscale).
        t % Capture time in seconds
        cameraModel % an instance of the camera model. 
        imustate %IMUSTATE Stores the state of this keyframe.
        
        ID = -1 %INT unique keyframe id. This id is used to associate landmarks and measurements with keyframe.
        
        isKeyframe = false; % this flag classifies this frame as a keyframe.
        
        landmarks = {}; %In this method landmarks are represented in the frame from which they were first observed (empty if not keyframe).
        
    end
    
    methods
        function obj = Frame(image, t, cameraModel, imuState)
            % constructs a frame object.
            obj.raw_image = image;
            obj.t = t;
            obj.cameraModel = cameraModel;
            
            if (nargin > 3)
                obj.imustate = imuState;
            else
                obj.imustate = IMUState();
            end
        end
    end
end

