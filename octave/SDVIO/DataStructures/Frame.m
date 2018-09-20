classdef Frame
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (mono8).
        time % Capture time in seconds
        focalLength % [fx, fy]
        principalPoint % [cx, cy]
        distortionCoefficients % [k1,....,kn] radial distortion coefficients of the eqidistant camera model used in kalibr.
        
        state %IMUSTATE Stores the state of this keyframe.
        
    end
    
    methods
        function obj = Frame(image, t, f, c, d, imuState)
            % constructs a frame object.
            obj.raw_image = image;
            obj.time = t;
            obj.principalPoint = c;
            obj.focalLength = f;
            obj.distortionCoefficients = d;
            obj.state = imuState;
        end
    end
end

