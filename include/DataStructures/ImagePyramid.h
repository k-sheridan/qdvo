#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <GlobalDefinitions.h>

namespace QDVO {
class ImagePyramid
{
public:
    /**
     * creates a new image pyramid from a base image.
     */
    ImagePyramid(int levels = 1);

    cv::Mat& getImage(const size_t level = 0);

    void generate(const cv::Mat& baseImage);

    size_t levels(){return this->imageLevels.size();}

protected:
    std::vector<cv::Mat> imageLevels; // level 0 is full resolution.

};
}

