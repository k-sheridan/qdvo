classdef Frame
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (mono8).
        time % Capture time in seconds
        cameraModel % an instance of the camera model. 
        imustate %IMUSTATE Stores the state of this keyframe.
        
        id %INT unique keyframe id. This id is used to associate landmarks and measurements with keyframe.
        
        isKeyframe = false; % this flag classifies this frame as a keyframe.
        
        landmarks = {}; %In this method landmarks are represented in the frame from which they were first observed (empty if not keyframe).
        
    end
    
    methods
        function obj = Frame(image, t, imuState)
            % constructs a frame object.
            obj.raw_image = image;
            obj.time = t;
            
            if (nargin > 2)
                obj.imustate = imuState;
            else
                obj.imustate = IMUState();
            end
        end
    end
end

