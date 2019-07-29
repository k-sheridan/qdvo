#include "Patch.h"

QDVO::Patch::Patch()
{

}

QDVO::Patch::Patch(cv::Mat& rawPatchData, float patchMean, float patchStdDev)
{
    assert(rawPatchData.type() == CV_8U);

    Eigen::Map<Eigen::Matrix<uint8_t, PATCH_WIDTH, PATCH_WIDTH, Eigen::RowMajor>> testPatchRaw( rawPatchData.ptr<uint8_t>() );

    this->data = testPatchRaw.cast<float>();
    this->patchMean = patchMean;
    this->patchStdDev = patchStdDev;

    this->zeroMeanMatrix = (this->data.array() - patchMean).eval();

    this->initialized = true;
    this->level = 0;
}

QDVO::Patch::Patch(cv::Mat& rawPatchData)
{
    assert(rawPatchData.type() == CV_8U);

    Eigen::Map<Eigen::Matrix<uint8_t, PATCH_WIDTH, PATCH_WIDTH, Eigen::RowMajor>> testPatchRaw( rawPatchData.ptr<uint8_t>() );

    this->data = testPatchRaw.cast<float>();

    this->patchMean = this->data.sum() / (PATCH_WIDTH * PATCH_WIDTH);

    this->zeroMeanMatrix = (this->data.array() - patchMean).eval();

    this->patchStdDev = this->zeroMeanMatrix.sum() / (PATCH_WIDTH * PATCH_WIDTH);

    this->initialized = true;
    this->level = 0;

}
