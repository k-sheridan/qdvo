#pragma once

#include <algorithm>
#include <memory>
#include <unordered_set>
#include <vector>

#include "GlobalDefinitions.h"
#include "Patch.h"
#include "RadialSearchPattern.h"
#include "SpatialMap.h"
#include "Types.h"

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
  struct PotentialCorrespondence {
    SCALAR_TYPE score = -1;  // match score.

    bool initialized() { return score != -1; }
    void reset() { score = -1; }
  };

  /// The warped template patch to be used for the creation of the
  /// correspondence distribution.
  Patch warpedPatch;

  /// Serves as a method for finding nearest neighbors.
  SpatialMap<PotentialCorrespondence> correspondenceMap;

  /// Is this correspondence distribution currently not being used.
  bool initialized = false;

  /// The landmark this correspondence distribution represents an observation
  /// of.
  LandmarkMap::key_type landmarkKey;

  /// Width and height of the image.
  const int width, height;

  CorrespondenceDistribution(
      unsigned width, unsigned height,
      std::shared_ptr<const RadialSearchPattern> patternPtr);

  /**
   * Will perform an initial radial search for potential correspondences to get
   * an idea of the structure of the raw patch comparison function.
   * @return Number of valid potential correspondences during initialization.
   */
  int initializeDistribution(
      CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey,
      const Eigen::Vector2i& centerPixel, const int floodRadius,
      std::shared_ptr<QDVO::PatchComparer> patchComparerPtr,
      QDVO::Patch warpedPatch);

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
   * Search the correspondence distribution for close by potential
   * correspondence distributions
   * @return vector of z - centerPixel, vector of scores associated to the
   * errors
   */
  void search(CameraModel& cameraModel, Frame& frame,
              const QDVO::Vector2& centerPixel, const unsigned searchRadius,
              bool minimalSearch, std::vector<QDVO::Vector2>& errors,
              std::vector<SCALAR_TYPE>& scores);

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

 private:
  // pre-allocated quantities.
  std::vector<SCALAR_TYPE> scoreArray, expScoreArray;
  std::vector<QDVO::Vector2> errorArray, weightedErrorArray;

  std::shared_ptr<const QDVO::RadialSearchPattern>
      radialSearchPattern;  // shared among all correspondence distributions.
                            // NOT TO BE MODIFIED!
  std::shared_ptr<QDVO::PatchComparer> patchComparer;
};

}  // namespace QDVO
