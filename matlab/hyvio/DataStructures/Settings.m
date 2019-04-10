classdef Settings < handle
    %SETTINGS A structure for holding all the settings for this VIO impl.
    
    properties
  
        maximumFramesStored = 200;
        
        patchHalfSize = 5; % the patch radius used for comparing.
        searchRadius = 10; % The radius which the pixel resolution patch matcher searches.
        
        initialDepth = 2.5; % the depth every landmark is initialized at.
        
        nFeaturesDesired = 400; % the number of features which are extracted for each keyframe.

        featureSeparation = 5 % the distance between feature desired (used for spatial sampling).
        activationFeatureSeparation = 10; % the distance between features during activation
        
        % from the DSO keyframe selection criteria.
        weightAvgPixelFlow = 0.04;
        weightAvgTranslationalFlow = 0.12;
        
        % EPIPOLAR DEPTH ESTIMATOR
        % the minimum depth, maximum depth, and resolution use din the
        % epipolar depth estimator.
        minimumDepth = 0.1;
        maximumDepth = 10;
        resolution = 200;
        maximumHypotheses = 10; % the maximum number of matches for a epipolar depth estimator to be called initialized
        finalDepthSearchResolution = 0; % after the epipolar depth estimator is initialized, search within its stdev with this resolution for a more accurate estimate.
        epipolarVarianceScale = 100; % this is the number which scales the variance of the epipolar depth estimate.
        maximumAttempts = 6; % this is the maximum number of times the epipolar depth estimator can be ran.
        
        
        minimumNormalizedMatchCorrelation = 0.95; % the threshold where a match is called good enough.
        correlationUniquenessThreshold = 0; % the minimum absolute difference between correlations. Set to 0 to skip this step.
        
        pixelOutlierThreshold = 3; % if after running the SWE any landmark observation error is above this threshold, marginalize it.
        maxFailedCorrespondences = 10; % maximum number of correspondence failures in a row before a landmark is marginalized
        huberWidth = 1;
        
        nActiveLandmarks = 300; % the number of landmarks which are to be active.
        minimumActiveLandmarks = 100; % the minimum feature number in a frame.
        windowSize = 7; % the SWE window size (number of keyframes in the window).
        
        patchComparison = 'ZNCC'; % types: 'ZNCC', 'BRIEF'
        
        % Feature Detection
        minimumNormalizedGradientMagnitude = 0.0153*2;

        medianFilterSize = 3;
        
        
        initial_T_camFromImu;
        initial_accelBias;
        initial_gyroBias;
        initial_accelScale;
        initial_gyroScale;
    end
    
    methods
        
    end
end

