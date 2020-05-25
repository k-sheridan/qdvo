#pragma once

#include <numeric>

#include "Types.h"

namespace QDVO {

class Graph;
class Frame;
class Landmark;
class PatchComparer;
class PatchWarper;

/**
 * Performs linear search for the inverse depth of a landmark.
 */
class EpipolarDepthEstimator {
 public:
  EpipolarDepthEstimator() = default;
  /// flag which means that the landmark depth has been succesfully initialized.
  bool initialized = false;

  double error = std::numeric_limits<double>::max();

  /// Keeps track of the number of attempted updates.
  int attempts = 0;

  /// Number of hypotheses in the previous best update.
  int hypotheses = 0;

  /// Return whether the landmark was seen at least once during an update.
  bool observedAtLeastOnce() {
    return error < std::numeric_limits<double>::max();
  }

  /**
   * Searches for the inverse depth of a landmark using a target keyframe. It is
   * assumed that the target keyframe pose has already been estimated.
   *
   * px(z) = Pi(T_target_source * (u * z))
   * d/dz px(z) = J_cameraModel * J_homogenousProjection * (R_target_source * u)
   * = J_z dz = resolution / norm(J_z)
   *
   * @param sourceKeyframe The keyframe which hosts the landmark.
   * @param targetKeyframe The keyframe used to search for the dinv in.
   * @param landmark The key of the landmark to estimate inverse depth for.
   */
  void update(Graph& graph, Frame& sourceKeyframe, Frame& targetKeyframe,
              Landmark& landmark, PatchComparer& patchComparer,
              PatchWarper& patchWarper);
};

}  // namespace QDVO
