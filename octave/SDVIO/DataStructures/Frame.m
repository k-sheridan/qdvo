classdef Frame
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (mono8).
        time % Capture time in seconds
        
        state %IMUSTATE Stores the state of this keyframe.
        
    end
    
    methods
        function obj = Frame(image, t, imuState)
            % constructs a frame object.
            obj.raw_image = image;
            obj.time = t;
            
            if (nargin > 2)
                obj.state = imuState;
            else
                obj.state = IMUState();
            end
        end
    end
end

