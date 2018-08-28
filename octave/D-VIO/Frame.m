classdef Frame
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (mono8).
        focalLength % [fx, fy]
        principalPoint % [cx, cy]
        distortionCoefficients % [k1,....,kn] radial distortion coefficients of the eqidistant camera model used in kalibr.
    end
    
    methods
        function brightness = subPixelIntensity(px)
            % Compute the sub pixel brightness with linear interpolation.
            lower = floor(px);
            upper = ceil(px);
            
            grad = upper - lower; % Brightness / Pixel (foward derivative).
            
            delta = px - lower;
            
            % 1st order Taylor series of the image near the pixel desired.
            brightness = lower + grad * delta';
        end
        
        
    end
end

