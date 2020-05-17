#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>
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

    Patch(cv::Mat& rawPatchData, float patchMean, float patchStdDev);

    Patch(cv::Mat& rawPatchData, float patchMean);

    Patch(cv::Mat& rawPatchData);

    Patch(Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH>& rawPatchData);

    Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH>& getImageData(){return data;}

    Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH>& getZeroMeanImageMatrix(){return zeroMeanMatrix;}

    float& getStdDev(){return patchStdDev;}

    float& getSumZeroMeanSquared(){return sumZeroMeanSquared;}

    float& getMean(){return patchMean;}

    int& getLevel(){return level;}

private:

    int level; // the image level this patch was created at. (0 = full resolution).
    bool initialized = false;
    Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH> data; // the pixel data.

    Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH> zeroMeanMatrix; // the mean shifted data, computed iff necessary
    float patchMean, patchStdDev, sumZeroMeanSquared;
};
}

