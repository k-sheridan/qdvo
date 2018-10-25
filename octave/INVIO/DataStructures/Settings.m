classdef Settings
    %SETTINGS A structure for holding all the settings for this VIO impl.
    
    properties
        nFeaturesDesired = 300; % the number of features which are desired for a given keyframe.
        patchHalfSize = 3; % the patch radius used for comparing.
        searchRadius = 10; % The radius which the pixel resolution patch matcher searches.
        
        maxBufferedFrames = 100; % the maximum size the frame buffer can be.
        
        initial_T_camFromImu;
        initial_accelBias;
        initial_gyroBias;
        initial_accelScale;
        initial_gyroScale;
    end
    
    methods
        
    end
end

