#ifndef COMPLETEIMAGEPYRAMID_H
#define COMPLETEIMAGEPYRAMID_H

#include <GlobalDefinitions.h>
#include <ImagePyramid.h>
#include <algorithm>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Core>

namespace QDVO {

/*
 * Contains an image pyramid with a standard deviation table, and gradients.
 */
class CompleteImagePyramid
{
public:
    /*
     * normal image pyramid constructor except now the local standard deviations are now computed.
     */
    CompleteImagePyramid(const int levels = 1);

    void generate(const cv::Mat& baseImage);

    float getMean(const cv::Point2f& px){return this->getColorSubpix(this->localMeanLUT, cv::Point2f(px.x / this->colStride, px.y / this->rowStride));}
    float getStdDev(const cv::Point2f& px);

    QDVO::ImagePyramid image;

//private:

    void setupTables(const cv::Mat& baseImage);

    float getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
            int x = (int)pt.x;
            int y = (int)pt.y;

            int x0 = cv::borderInterpolate(x,   img.cols, cv::BORDER_REFLECT_101);
            int x1 = cv::borderInterpolate(x+1, img.cols, cv::BORDER_REFLECT_101);
            int y0 = cv::borderInterpolate(y,   img.rows, cv::BORDER_REFLECT_101);
            int y1 = cv::borderInterpolate(y+1, img.rows, cv::BORDER_REFLECT_101);

            float a = pt.x - (float)x;
            float c = pt.y - (float)y;

            return ((img.at<float>(y0, x0) * (1.f - a) + img.at<float>(y0, x1) * a) * (1.f - c)
                                   + (img.at<float>(y1, x0) * (1.f - a) + img.at<float>(y1, x1) * a) * c);
    }

    // these tables are generated from the top down through upsampling.
    cv::Mat localStandardDeviationLUT; // used to speed up the patch comparison metrics.
    cv::Mat localMeanLUT;
    float rowStride, colStride;
};
}

#endif // COMPLETEIMAGEPYRAMID_H
