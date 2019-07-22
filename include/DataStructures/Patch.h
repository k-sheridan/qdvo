#ifndef PATCH_H
#define PATCH_H

#include <opencv2/core.hpp>
#include "GlobalDefinitions.h"
#include "Types.h"

namespace QDVO {
/*
 * stores the information which defines a feature. Typically, this is just a small region of interest around the feature.
 * This is usually used as a way of storing a warped patch.
 */
class Patch
{
public:
    Patch();

    Patch(cv::Mat& rawPatchData, QDVO::Vector2 centerPixel, SCALAR_TYPE patchMean, SCALAR_TYPE patchStdDev);

    Patch(cv::Mat& rawPatchData);

    cv::Mat& getImageData(){return data;}

    cv::Mat& getZeroMeanImageData(){return zeroMeanData;}

    SCALAR_TYPE& getStdDev(){return patchStdDev;}

    SCALAR_TYPE& getMean(){return patchMean;}

    int& getLevel(){return level;}

private:

    int level; // the image level this patch was created at. (0 = full resolution).
    bool initialized = false;
    cv::Mat data; // the pixel data.

    cv::Mat zeroMeanData; // the mean shifted data, computed iff necessary
    SCALAR_TYPE patchMean, patchStdDev;
};
}

#endif // PATCH_H
