#pragma once

#include <opencv4/opencv2/core.hpp>
#include <Feature.h>
#include <Frame.h>

class FeatureDetector
{
public:
    FeatureDetector();

    std::vector<Feature> detectFeatures(const Frame& frame);
};

