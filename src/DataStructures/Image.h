#pragma once

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "DataStructures/Patch.h"
#include "GlobalDefinitions.h"
#include "Types.h"

namespace QDVO {

typedef float ImageIntensityType;
typedef Eigen::Matrix<ImageIntensityType, Eigen::Dynamic, Eigen::Dynamic>
    ImageType;

class Image {
 public:
  Image(cv::Mat& cvImage) { cv::cv2eigen(cvImage, this->image); }
  Image() {}

  cv::Mat toOpenCVImage() {
    cv::Mat img;
    cv::eigen2cv(this->image, img);
    return img;
  }

  ImageType& getImageData() { return image; }
  const ImageType& getImageData() const { return image; }

  /// Pixel order: x, y.
  /// This function will handle the mapping between pixel coordinates and row,
  /// column access.
  QDVO::ImageIntensityType at(const Eigen::Vector2i& px) const {
    return image(px.y(), px.x());
  }

  int rows() const { return image.rows(); }
  int cols() const { return image.cols(); }

  QDVO::Result<ImageIntensityType> getSubPixelIntensity(QDVO::Vector2 px) const;

  QDVO::Result<Patch> getSubPixelPatch(QDVO::Vector2 centerPixel,
                                       int patchWidth = PATCH_WIDTH) const;

 private:
  ImageType image;
};
}  // namespace QDVO
