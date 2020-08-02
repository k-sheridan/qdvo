#include "Frame.h"

#include <opencv2/imgproc/imgproc.hpp>

#include "Config.h"
#include "Profiling.h"

QDVO::Frame::Frame() {
  this->status = FrameStatus::INACTIVE;
  this->imagePyr = QDVO::ImagePyramid(IMAGE_PYRAMID_LEVELS);
}

void QDVO::Frame::updateImage(cv::Mat &baseImage) {
  // set the max intensity
  switch (baseImage.type()) {
    case CV_8U:
      this->maxImageIntensity = 255;
      break;

    case CV_16U:
      this->maxImageIntensity = 65535;
      break;

    // Just add the maximum value for the type.
    default:
      throw std::runtime_error("image type not supported.");
  }

  // Median filter.
  if (config->medianFilterImages) {
    cv::Mat filteredBaseImage;
    {
      PROFILE("medianFilter");
      cv::medianBlur(baseImage, filteredBaseImage, 3);
    }
    this->imagePyr.generate(filteredBaseImage);
  } else {
    this->imagePyr.generate(baseImage);
  }
}

void QDVO::Frame::reset() {
  // remove the landmarks
  this->landmarkKeys.clear();

  // reset all correspondence distributions
  this->resetCorrespondenceDistributions();

  status = FrameStatus::INACTIVE;

  initialized = false;
}
