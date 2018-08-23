classdef Frame
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (mono).
    end
    
    methods
        function brightness = subPixelBrightness(px)
            % Compute the sub pixel brightness with linear interpolation.
            lower = floor(px);
            upper = ceil(px);
            
            grad = upper - lower; % Brightness / Pixel.
            
            delta = px - lower;
            
            % Taylor series of the image.
            brightness = lower + grad * delta';
        end
        
        
    end
end

