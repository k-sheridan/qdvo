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
     *
     * If the level count is -1, the image pyramid will be built as high as it can go until the image dimensions are no longer divisible by 2.
     */
    CompleteImagePyramid(const int levels = -1, const int standardDeviationTableResolution = 16);

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
    QDVO::ImagePyramid standardDeviationTables; // used to speed up the patch comparison metrics.
    QDVO::ImagePyramid meanTables;
};
}

#endif // COMPLETEIMAGEPYRAMID_H
