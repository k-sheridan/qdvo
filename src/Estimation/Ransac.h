#pragma once

#include <random>

#include "DataStructures/Graph.h"
#include "Optimizer/Containers.h"
#include "Optimizer/Variables/SE3.h"

namespace QDVO {
class RansacPoseInitializer {
 public:
  struct RansacResult {
    QDVO::SE3 pose;
    QDVO::Scalar resultWhitenedSqError;
  };

  struct RansacSettings {
    int iterations = 1000;
    QDVO::Scalar translationMagnitude = 0.5;
    QDVO::Scalar rotationMagnitudeRad = 0.524;
  };

  RansacPoseInitializer() : dis(-1.0, 1.0) {
    // Make deterministic.
    gen.seed(1);
  }

  /// Compute the residual for a given correspondence distribution.
  QDVO::Result<QDVO::Vector2> computeWhitenedSquaredError(
      const QDVO::Graph& graph, const QDVO::Frame& frame,
      QDVO::CorrespondenceDistribution& correspondenceDistribution,
      const QDVO::SE3& frameImuState,
      bool allowUninitializedCorrespondenceDistribution = false) {}

  /**
   * @param graph The main graph to fit the frame on.
   * @param frame The frame to fit.
   * @param settings Ransac settings
   */
  RansacResult run(QDVO::Graph& graph, QDVO::Frame& frame,
                   const RansacSettings& settings) {}

 private:
  double generateRandomValue() { return dis(gen); }

  std::mt19937 gen;
  std::uniform_real_distribution<double> dis;
};
}  // namespace QDVO
