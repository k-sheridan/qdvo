#pragma once

#include <deque>
#include <opencv2/core.hpp>

#include "CameraModel.hpp"
#include "CorrespondenceDistribution.h"
#include "IMUState.h"
#include "ImagePyramid.h"
#include "Landmark.h"
#include "Optimizer/SlotMap.h"
#include "Types.h"
#include "Logging.h"

namespace QDVO {
class Frame {
 public:
  Frame();

  /// Stores a key to the camera model used for projecting points into the
  /// frame.
  CameraModelMap::key_type cameraModelKey;

  /// A key pointing the the imu to camera transform for this frame.
  ExtrinsicMap::key_type extrinsicKey;

  /// Stores the state of this frame.
  IMUState imustate;

  /// Stores the image measurement for this frame.
  ImagePyramid imagePyr;

  /// Array of keys to landmarks hosted in this frame.
  std::vector<LandmarkMap::key_type> landmarkKeys;

  /**
   * A preallocated array of correspondence distributions for this frame.
   * This array should never be shrunk. the correspondence distributions can be
   * uninitialized though.
   */
  CorrespondenceDistributionMap correspondenceDistributions;

  /// Flag to mark whether the frame has ever been setup.
  bool initialized = false;

  /// enum representing the state of this frame.
  enum FrameStatus { INACTIVE, ACTIVE, MARGINALIZED } status;

  void updateImage(cv::Mat &baseImage);

  int maxIntensity() { return this->maxImageIntensity; }

  /**
   * resets all members of this frame while leaving the memory used by them
   * allocated.
   */
  void reset();

  void resetCorrespondenceDistributions() {
    LOG_TRACE("Clearing correspondence distributions.");
    correspondenceDistributions.clear();
  }

 private:
  /// The maximum value any element in the image can be.
  int maxImageIntensity;
};

}  // namespace QDVO
