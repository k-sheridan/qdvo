#pragma once

#include <enoki/array.h>
#include <enoki/dynamic.h>

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>

#include "GlobalDefinitions.h"
#include "Patch.h"
#include "RadialSearchPattern.h"
#include "SpatialMap.h"
#include "Types.h"

/**
 * Vectorizable SoA container for the potential correspondences.
 */
namespace QDVO {
template <typename Value>
struct PotentialCorrespondence {
  using Pixel = enoki::Array<enoki::float32_array_t<Value>, 2>;

  /// Pixel of the correspondence.
  Pixel pixel;
  /// Score of correspondence.
  Value score;

  ENOKI_STRUCT(PotentialCorrespondence, pixel, score)
};
}  // namespace QDVO
ENOKI_STRUCT_SUPPORT(QDVO::PotentialCorrespondence, pixel, score)

/**
 * Vectorizable SoA container for the nearby potential correspondences.
 */
namespace QDVO {
template <typename Value>
struct NearbyPotentialCorrespondence {
  using RelativePixel = enoki::Array<enoki::float32_array_t<Value>, 2>;

  /// Pixel offset of the correspondence.
  RelativePixel pixelOffset;
  /// Score of correspondence.
  Value score;
  /// Squared distance of the correspondence.
  Value squaredDistance;

  ENOKI_STRUCT(NearbyPotentialCorrespondence, pixelOffset, score,
               squaredDistance)
};
}  // namespace QDVO
ENOKI_STRUCT_SUPPORT(QDVO::NearbyPotentialCorrespondence, pixelOffset, score,
                     squaredDistance)

namespace QDVO {

class Frame;
class CameraModel;
class PatchComparer;

/**
 * The correspondence distribution is modeled as a gaussian mixture model. The
 * means are of pixel resolution and have associated scores. This class provides
 * all the functionality necessary for efficiently adding to and evaluting the
 * gaussian mixture model. It is meant to be preallocated once and held until
 * the algorithm completes.
 */
class CorrespondenceDistribution {
 public:
  /// Scalar used for enoki.
  using EnokiScalar = float;

  /// The warped template patch to be used for the creation of the
  /// correspondence distribution.
  Patch warpedPatch;

  /// Is this correspondence distribution currently not being used.
  bool initialized = false;

  /// The landmark this correspondence distribution represents an observation
  /// of.
  LandmarkMap::key_type landmarkKey;

  /// Covariance matrix of this correpsponence distributions.
  Eigen::Matrix<QDVO::Scalar, 2, 2> Sigma;

  /// SoA of potential correspondences.
  PotentialCorrespondence<enoki::DynamicArray<enoki::Packet<EnokiScalar>>>
      potentialCorrespondences;

  /// SoA of nearby potential correspondences.
  NearbyPotentialCorrespondence<enoki::DynamicArray<enoki::Packet<EnokiScalar>>>
      nearbyPotentialCorrespondences;

  CorrespondenceDistribution();

  /**
   * Will perform an initial radial search for potential correspondences to get
   * an idea of the structure of the raw patch comparison function.
   * @return Number of valid potential correspondences during initialization.
   */
  int initializeDistribution(
      CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey,
      const Eigen::Vector2i& centerPixel, const int floodRadius,
      QDVO::Patch warpedPatch,
      float threshold = POTENTIAL_CORRESPONDENCE_THRESHOLD);

  /// Given an error and score vector, fit a gaussian.
  Eigen::Matrix<QDVO::Scalar, 2, 2> fitGaussian();

  /**
   * Efficiently evaluates the gradient of the negative log likelihood of the
   * gaussian mixture model described by this class.
   */
  QDVO::Result<QDVO::Vector2> computeResidual(
      CameraModel& cameraModel, Frame& frame,
      const Eigen::Matrix<SCALAR_TYPE, 2, 1>& px_0);

  /**
   * clears all potential correspondences while retaining allocated memory, and
   * sets the correspondence distribution into a dormant state.
   */
  void reset();

  /**
   * Computes a matrix which stores the scores in a region of the correspondence
   * distribution.
   * @param center Center pixel of the region.
   * @param dimensions Width and Height of the region.
   * @return Scores in a region of the correspondence distribution. If the score
   * is -1, it does not exist.
   */
  Eigen::Matrix<SCALAR_TYPE, Eigen::Dynamic, Eigen::Dynamic> extractScores(
      Eigen::Vector2i center, Eigen::Vector2i dimensions);
};

}  // namespace QDVO
