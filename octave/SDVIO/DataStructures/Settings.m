classdef Settings
    %SETTINGS A structure for holding all the settings for this VIO impl.
    
    properties
        nFeaturesDesired = 300; % the number of features which are desired for a given keyframe.
        patchHalfSize = 3; % the patch radius used for comparing.
        searchRadius = 10; % The radius which the pixel resolution patch matcher searches.
    end
    
    methods
        
    end
end

