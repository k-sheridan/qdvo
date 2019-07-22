#include "Patch.h"

QDVO::Patch::Patch()
{

}

QDVO::Patch::Patch(cv::Mat& rawPatchData, QDVO::Vector2 centerPixel, SCALAR_TYPE patchMean, SCALAR_TYPE patchStdDev)
{
    assert(rawPatchData.type() == CV_8U);

    this->data = rawPatchData;
    this->patchMean = patchMean;
    this->patchStdDev = patchStdDev;

    cv::subtract(this->data, cv::Scalar(this->patchMean), this->zeroMeanData, cv::noArray(), CV_32S);
}

QDVO::Patch::Patch(cv::Mat& rawPatchData)
{
    assert(rawPatchData.type() == CV_8U);

    this->data = rawPatchData;

    cv::Scalar mn, stdev;
    cv::meanStdDev(this->data, mn, stdev);

    this->patchMean = mn(0);
    this->patchStdDev = stdev(0);

    cv::subtract(this->data, mn, this->zeroMeanData, cv::noArray(), CV_32S);
}
