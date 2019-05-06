#pragma once

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <GlobalDefinitions.h>

namespace QDVO {
class ImagePyramid
{
public:
    /**
     * creates a new image pyramid from a base image.
     */
    ImagePyramid(const cv::Mat& baseImage, int levels);

    cv::Mat& getImage(const int level = 0);

protected:
    std::vector<cv::Mat> imageLevels; // level 0 is full resolution.

};
}

