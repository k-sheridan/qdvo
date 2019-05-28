#pragma once

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <Feature.h>
#include <Frame.h>
#include <GlobalDefinitions.h>
#include <algorithm>


// Feature Detector Settings
#define N_FEATURES_DESIRED 400
#define N_SECTIONS 100
#define HARRIS_K  0.05
#define HARRIS_WIDTH 3
#define EDGE_WEIGHT 0.1
#define INVARIANT_THRESHOLD  0.5
#define MINIMUM_NORMALIZED_GRADIENT_MAG 0.120

#define USE_SPATIAL_MASK true
#define SPATIAL_MASK_RADIUS 6

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

    struct FeatureCandidate{
        int x, y; // pixel position
        SCALAR_TYPE dxdx, dxdy, dydy; // structure tensor information
        SCALAR_TYPE det, trace; // determinant and trace of the structure tensor.
        SCALAR_TYPE gradientNorm;
        SCALAR_TYPE harris;
        SCALAR_TYPE score = 0; // stores the feature score which I have described in my paper.
    };

//private:
    cv::Mat dx, dy; // preallocated containers for the image gradients.
    cv::Mat dxdx, dydy, dxdy; // preallocated containers for structure tensors.

    cv::Mat spatialMask; // used to ensure no two features are too close to each other.
};
}

