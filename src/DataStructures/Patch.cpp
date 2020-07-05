#include "Patch.h"

#include <opencv2/core/eigen.hpp>

#include "Image.h"

QDVO::Patch::Patch() {}

QDVO::Patch::Patch(Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH>& rawPatchData,
                   int level) {
  this->data = rawPatchData;
  this->patchMean = this->data.sum() / (PATCH_WIDTH * PATCH_WIDTH);

  this->zeroMeanMatrix = (this->data.array() - patchMean).eval();

  this->sumZeroMeanSquared = this->zeroMeanMatrix.cwiseAbs2().sum();

  this->patchStdDev = this->sumZeroMeanSquared / (PATCH_WIDTH * PATCH_WIDTH);

  this->initialized = true;
  this->level = level;
}

QDVO::Patch::Patch(cv::Mat& rawPatchData, float patchMean, float patchStdDev) {
  assert(rawPatchData.type() == CV_8U);
  assert(rawPatchData.cols == PATCH_WIDTH && rawPatchData.rows == PATCH_WIDTH);

  // Eigen::Map<Eigen::Matrix<uint8_t, PATCH_WIDTH, PATCH_WIDTH,
  // Eigen::RowMajor>> testPatchRaw( rawPatchData.ptr<uint8_t>() );

  // this->data = testPatchRaw.cast<float>();
  cv2eigen(rawPatchData, this->data);

  this->patchMean = patchMean;
  this->patchStdDev = patchStdDev;

  this->sumZeroMeanSquared =
      this->patchStdDev * this->patchStdDev * (PATCH_WIDTH * PATCH_WIDTH);

  this->zeroMeanMatrix = (this->data.array() - patchMean).eval();

  this->initialized = true;
  this->level = 0;
}

QDVO::Patch::Patch(cv::Mat& rawPatchData, float patchMean) {
  assert(rawPatchData.type() == CV_8U);
  assert(rawPatchData.cols == PATCH_WIDTH && rawPatchData.rows == PATCH_WIDTH);

  // Eigen::Map<Eigen::Matrix<uint8_t, PATCH_WIDTH, PATCH_WIDTH,
  // Eigen::RowMajor>> testPatchRaw( rawPatchData.ptr<uint8_t>() );

  // this->data = testPatchRaw.cast<float>();

  cv2eigen(rawPatchData, this->data);

  this->patchMean = patchMean;

  this->zeroMeanMatrix = (this->data.array() - patchMean).eval();

  this->sumZeroMeanSquared = this->zeroMeanMatrix.cwiseAbs2().sum();

  this->patchStdDev = this->sumZeroMeanSquared / (PATCH_WIDTH * PATCH_WIDTH);

  this->initialized = true;
  this->level = 0;
}

QDVO::Patch::Patch(cv::Mat& rawPatchData) {
  assert(rawPatchData.type() == CV_8U);
  assert(rawPatchData.cols == PATCH_WIDTH && rawPatchData.rows == PATCH_WIDTH);

  cv::Mat temp;
  rawPatchData.convertTo(temp, CV_32F);
  Eigen::Map<Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH>, Eigen::RowMajor>
      testPatchRaw(temp.ptr<float>(), PATCH_WIDTH, PATCH_WIDTH);

  assert(testPatchRaw(9, 9) == temp.at<float>(9, 9));

  this->data = testPatchRaw.cast<float>();

  this->patchMean = this->data.sum() / (PATCH_WIDTH * PATCH_WIDTH);

  this->zeroMeanMatrix = (this->data.array() - patchMean).eval();

  this->sumZeroMeanSquared = this->zeroMeanMatrix.cwiseAbs2().sum();

  this->patchStdDev = this->sumZeroMeanSquared / (PATCH_WIDTH * PATCH_WIDTH);

  this->initialized = true;
  this->level = 0;
}
