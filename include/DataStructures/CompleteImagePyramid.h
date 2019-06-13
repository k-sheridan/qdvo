#ifndef COMPLETEIMAGEPYRAMID_H
#define COMPLETEIMAGEPYRAMID_H

#include <GlobalDefinitions.h>
#include <ImagePyramid.h>
#include <algorithm>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>

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


    typedef float StandardDeviationType;
    StandardDeviationType getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
        cv::Mat patch;
        cv::getRectSubPix(img, cv::Size(1,1), pt, patch);
        return patch.at<StandardDeviationType>(0,0);
    }


    QDVO::ImagePyramid image;

    // these tables are generated from the top down through upsampling.
    cv::Mat localStandardDeviationLUT; // used to speed up the patch comparison metrics.
    cv::Mat localMeanLUT;
};
}

#endif // COMPLETEIMAGEPYRAMID_H
