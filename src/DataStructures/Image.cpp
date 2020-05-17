#include "Image.h"

#include <opencv2/core/eigen.hpp>

QDVO::Result<QDVO::ImageIntensityType> QDVO::Image::getSubPixelIntensity(
    QDVO::Vector2 px) const {
  int x0 = (int)px.x();
  int y0 = (int)px.y();
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  if ((x0 >= (this->cols() - 1) || x0 < 0) ||
      (y0 >= (this->rows() - 1) || y0 < 0)) {
    // throw std::runtime_error("pixel out of bounds");
    return {};
  }

  float a = px.x() - (float)x0;
  float c = px.y() - (float)y0;

  return ((image(y0, x0) * (1.f - a) + image(y0, x1) * a) * (1.f - c) +
          (image(y1, x0) * (1.f - a) + image(y1, x1) * a) * c);
}

QDVO::Result<QDVO::Patch> QDVO::Image::getSubPixelPatch(
    QDVO::Vector2 centerPixel, int patchWidth) const {
  assert(patchWidth == PATCH_WIDTH);
  Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH> imageData;

  for (int deltaX = -PATCH_RADIUS; deltaX <= PATCH_RADIUS; ++deltaX) {
    for (int deltaY = -PATCH_RADIUS; deltaY <= PATCH_RADIUS; ++deltaY) {
      auto result =
          getSubPixelIntensity(centerPixel + QDVO::Vector2(deltaX, deltaY));
      if (!result.has_value()) {
        // Failed to get patch.
        return {};
      }
      // Insert the result into the imageData.
      imageData(deltaY + PATCH_RADIUS, deltaX + PATCH_RADIUS) =
          (result.value());
    }
  }
  return QDVO::Patch(imageData);
}
