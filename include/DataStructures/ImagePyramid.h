#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "GlobalDefinitions.h"

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

    size_t levels() const {return this->imageLevels.size();}

    float getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
            assert(img.type() == 0);
            int x0 = (int)pt.x;
            int y0 = (int)pt.y;
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            if ((x0 >= (img.cols+1) || x0 < 0) || (y0 >= (img.cols+1) || y0 < 0))
            {
                throw std::runtime_error("pixel out of bounds");
            }

            float a = pt.x - (float)x0;
            float c = pt.y - (float)y0;

            return ((img.at<uint8_t>(y0, x0) * (1.f - a) + img.at<uint8_t>(y0, x1) * a) * (1.f - c)
                                   + (img.at<uint8_t>(y1, x0) * (1.f - a) + img.at<uint8_t>(y1, x1) * a) * c);
    }

protected:
    std::vector<cv::Mat> imageLevels; // level 0 is full resolution.

};
}

