#include "PatchComparer.h"

QDVO::PatchComparer::PatchComparer() {}

QDVO::Result<SCALAR_TYPE> QDVO::PatchComparer::compare(QDVO::Patch& patch,
                                                       Frame& targetFrame,
                                                       Eigen::Vector2i& pixel) {
  cv::Rect roi(
      cv::Point2i(pixel(0), pixel(1)) - cv::Point2i(PATCH_RADIUS, PATCH_RADIUS),
      cv::Size2i(PATCH_WIDTH, PATCH_WIDTH));

  Eigen::Vector2i shift(PATCH_RADIUS, PATCH_RADIUS);
  Eigen::Vector2i tl = pixel - shift;
  Eigen::Vector2i br = pixel + shift;

  assert(patch.getStdDev() > 1e-8);
  assert(PATCH_WIDTH == 2 * PATCH_RADIUS + 1);

  QDVO::ImageType& targetImage = targetFrame.imagePyr.getImage().getImageData();

  if (tl(0) < 0 || tl(1) < 0 || br(0) >= targetImage.cols() ||
      br(1) >= targetImage.rows()) {
    return {};
  }

  // get the test patch from the image and compute its zero mean self.
  Eigen::Matrix<QDVO::ImageIntensityType, PATCH_WIDTH, PATCH_WIDTH> patchData =
      targetImage.block<PATCH_WIDTH, PATCH_WIDTH>(tl(1), tl(0));
  QDVO::Patch testPatch = QDVO::Patch(patchData);

  auto resultingScore = compare(patch, testPatch);

  return resultingScore;
}

QDVO::Result<SCALAR_TYPE> QDVO::PatchComparer::compare(QDVO::Patch& patch,
                                                       QDVO::Patch& testPatch) {
  // Check if any of the patches have are filled with the same value.
  if (patch.getSumZeroMeanSquared() < 1e-9 ||
      testPatch.getSumZeroMeanSquared() < 1e-9) {
    return {};
  }

  SCALAR_TYPE resultingScore =
      (patch.getZeroMeanImageMatrix().array() *
       testPatch.getZeroMeanImageMatrix().array())
          .sum() /
      sqrt(patch.getSumZeroMeanSquared() * testPatch.getSumZeroMeanSquared());

  return ((resultingScore + 1) / 2);
}
