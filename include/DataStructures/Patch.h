#ifndef PATCH_H
#define PATCH_H

#include <opencv2/core.hpp>
#include "GlobalDefinitions.h"

namespace QDVO {
/*
 * stores the information which defines a feature. Typically, this is just a small region of interest around the feature.
 * This is usually used as a way of storing a warped patch.
 */
class Patch
{
public:
    Patch();

    Patch(cv::Mat& rawPatchData);



private:

    int level; // the image level this patch was created at. (0 = full resolution).
    bool initialized = false;
    cv::Mat data; // the pixel data.

    cv::Mat zeroMeanData; // the mean shifted data, computed iff necessary
    SCALAR_TYPE patchMean, patchStdDev;
};
}

#endif // PATCH_H
