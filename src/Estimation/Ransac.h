#pragma once

#include <random>

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

  template <typename... ErrorTerms, typename... Variables>
  RansacResult run(ArgMin::ErrorTermContainer<ErrorTerms...>& errorTerms,
                   ArgMin::VariableContainer<Variables...>& variables,
                   const ArgMin::VariableKey<ArgMin::SE3>& poseKeyToSearch,
                   const QDVO::SE3& initialGuess,
                   const RansacSettings& settings) {}

 private:
  double generateRandomValue() { return dis(gen); }

  std::mt19937 gen;
  std::uniform_real_distribution<double> dis;
};
}  // namespace QDVO
