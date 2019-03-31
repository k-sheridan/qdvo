classdef Settings < handle
    %SETTINGS A structure for holding all the settings for this VIO impl.
    
    properties
  
        patchHalfSize = 5; % the patch radius used for comparing.
        searchRadius = 20; % The radius which the pixel resolution patch matcher searches.
        
        nFeaturesDesired = 200; % the number of features which are extracted for each keyframe.
        featureSeparation = 20; % the distance between landmarks desired (used for spatial sampling).
        
        % from the DSO keyframe selection criteria.
        weightAvgPixelFlow = 0.04;
        weightAvgTranslationalFlow = 0.12;
        
        % EPIPOLAR DEPTH ESTIMATOR
        % the minimum depth, maximum depth, and resolution use din the
        % epipolar depth estimator.
        minimumDepth = 1;
        maximumDepth = 20;
        resolution = 100;
        maximumHypotheses = 10; % the maximum number of matches for a epipolar depth estimator to be called initialized
        finalDepthSearchResolution = 0; % after the epipolar depth estimator is initialized, search within its stdev with this resolution for a more accurate estimate.
        epipolarVarianceScale = 36; % this is the number which scales the variance of the epipolar depth estimate.
        maximumAttempts = 2; % this is the maximum number of times the epipolar depth estimator can be ran.
        
        
        minimumNormalizedMatchCorrelation = 0.95; % the threshold where a match is called good enough.
        correlationUniquenessThreshold = 0; % the minimum absolute difference between correlations. Set to 0 to skip this step.
        
        nActiveLandmarks = 120; % the number of landmarks which are to be active.
        minimumActiveLandmarks = 90; % the minimum feature number in a frame.
        windowSize = 7; % the SWE window size (number of keyframes in the window).
        
        patchComparison = 'ZNCC'; % types: 'ZNCC', 'BRIEF'
        
        
        initial_T_camFromImu;
        initial_accelBias;
        initial_gyroBias;
        initial_accelScale;
        initial_gyroScale;
    end
    
    methods
        
    end
end

