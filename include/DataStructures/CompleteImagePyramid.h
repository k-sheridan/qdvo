#ifndef COMPLETEIMAGEPYRAMID_H
#define COMPLETEIMAGEPYRAMID_H

#include <GlobalDefinitions.h>
#include <ImagePyramid.h>

namespace QDVO {

/*
 * Contains an image pyramid with a standard deviation table, and gradients.
 */
class CompleteImagePyramid
{
public:
    /*
     * normal image pyramid constructor except now the image gradients and local standard deviations are now computed.
     */
    CompleteImagePyramid(const int levels, const int standardDeviationTableResolution = 16);

    void generate(const cv::Mat& baseImage);


    typedef float StandardDeviationType;
    StandardDeviationType getColorSubpix(const cv::Mat& img, cv::Point2f pt)
    {
        cv::Mat patch;
        cv::getRectSubPix(img, cv::Size(1,1), pt, patch);
        return patch.at<StandardDeviationType>(0,0);
    }


    QDVO::ImagePyramid image, dx, dy;

    cv::Mat standardDeviationTable; // used to speed up the patch comparison metrics.
};
}

#endif // COMPLETEIMAGEPYRAMID_H
