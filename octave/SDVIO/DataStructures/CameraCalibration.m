classdef CameraCalibration
    % stores the camera calibration 
    
    properties
        focalLength % [fx, fy]
        principalPoint % [cx, cy]
        distortionCoefficients % [k1,....,kn] radial distortion coefficients of the eqidistant camera model used in kalibr.
        imageSize; %[width, height]
    end
    
    methods
        
    end
end

