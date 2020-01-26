#pragma once

#include "GlobalDefinitions.h"
namespace QDVO {

class Settings
{
public:

    // global settings file
    int nKeyframes = 7;

    // from the DSO keyframe selection criteria.
    double weightAvgPixelFlow = 0.04;
    double weightAvgTranslationalFlow = 0.12;

    // Settings for the sliding window estimator.
    double huberWidth = 1;
    double pixelOutlierThreshold = 3;

    // feature detection settings
    struct FeatureDetectionSettings {
        int nFeaturesDesired = 400;
        int nSections = 20;
        SCALAR_TYPE harrisK = 0.05;
        SCALAR_TYPE edgeWeight = 0.1;
        SCALAR_TYPE invariantThreshold = 0.1;
        SCALAR_TYPE minimumNormalizedGradientMagnitude = 0.0306;
    } feature_detection;

    // Epipolar depth estimator settings.
    struct EpipolarDepthEstimatorSettings {
	int maximumAttempts = 6;
	double maximumError = 0.1;
	int maximumHypotheses = 5;
	double minimumDepth = 0.4; 
	double maximumDepth = 20;
	double resolution = 0.5;
    } epipolar_depth_estimator;
};
}

