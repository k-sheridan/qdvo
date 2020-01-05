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

    // feature detection settings
    struct FeatureDetectionSettings {
        int nFeaturesDesired = 400;
        int nSections = 100;
        SCALAR_TYPE harrisK = 0.05;
        SCALAR_TYPE edgeWeight = 0.1;
        SCALAR_TYPE invariantThreshold = 0.1;
        SCALAR_TYPE minimumNormalizedGradientMagnitude = 0.0306;
    } feature_detection;
};
}

