#pragma once

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <Feature.h>
#include <Frame.h>

namespace  QDVO {
class FeatureDetector
{
public:
    FeatureDetector();

    /**
     * Seeks to find a set of "trackable" features such that the set of features is well distributed spatially.
     * Both edges and corners will be called features.
     */
    std::vector<QDVO::Feature> detectFeatures(const Frame& frame);

private:
    cv::Mat dx, dy; // preallocated containers for the image gradients.
};
}

