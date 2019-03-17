classdef Settings < handle
    %SETTINGS A structure for holding all the settings for this VIO impl.
    
    properties
        nFeaturesDesired = 200; % the number of features which are desired for a given keyframe.
        patchHalfSize = 5; % the patch radius used for comparing.
        searchRadius = 10; % The radius which the pixel resolution patch matcher searches.
        
        minimumNormalizedMatchCorrelation = 0.9; % the threshold where a match is called good enough.
        correlationUniquenessThreshold = 0; % the minimum absolute difference between correlations. Set to 0 to skip this step.
        
        trans2DepthRatio = 0.05; % the ratio of translation from last keyframe to avg scene depth when a keyframe is made.
        minimumFeatures = 40; % th minimum feature number in a frame.
        
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

