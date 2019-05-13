#include "FeatureDetector.h"

QDVO::FeatureDetector::FeatureDetector()
{

}


std::vector<QDVO::Feature> QDVO::FeatureDetector::detectFeatures(const Frame& frame)
{
    // First, compute the image gradients.
    cv::Sobel( frame.image, this->dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( frame.image, this->dy, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    std::vector<QDVO::Feature> features;

    return features;
}
