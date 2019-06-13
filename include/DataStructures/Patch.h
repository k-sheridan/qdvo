#ifndef PATCH_H
#define PATCH_H

#include <opencv2/core.hpp>

namespace QDVO {
/*
 * stores the information which defines a feature. Typically, this is just a small region of interest around the feature.
 * This is usually used as a way of storing a warped patch.
 */
class Patch
{
public:
    Patch();

    int level; // the image level this patch was created at. (0 = full resolution).

    cv::Mat image; // the pixel data.
};
}

#endif // PATCH_H
