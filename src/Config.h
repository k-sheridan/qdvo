#pragma once

#include <memory>

#include "Logging.h"
#include "GlobalDefinitions.h"

namespace QDVO {
class Config {
 public:
  struct Parameters {
    /// global settings file
    int nKeyframes = 7;

    /// from the DSO keyframe selection criteria.
    double weightAvgPixelFlow = 0.04;
    double weightAvgTranslationalFlow = 0.12;

    /// Settings for the sliding window estimator.
    double huberWidth = 1;
    double pixelOutlierThreshold = 3;

    /// feature detection settings
    struct FeatureDetectionSettings {
      int nFeaturesDesired = 400;
      int nSections = 20;
      double harrisK = 0.05;
      double edgeWeight = 0.1;
      double invariantThreshold = 0.1;
      double minimumNormalizedGradientMagnitude = 0.0306;
    } feature_detection;

    /// Epipolar depth estimator settings.
    struct EpipolarDepthEstimatorSettings {
      int maximumAttempts = 6;
      double maximumError = 0.2;
      int maximumHypotheses = 10;
      double minimumDepth = 0.4;
      double maximumDepth = 20;
      double resolution = 1;
    } epipolar_depth_estimator;
  };

  /// Set the parameters
  void setParameters(const Parameters& params) {
    if (initialized) {
      LOG_ERROR("Tried to overwrite the config file.");
      abort();
    }
    initialized = true;
    parameters = params;
  }

  /// Access the parameters.
  const Parameters* operator->() const {
    LOG_TRACE_IF(!initialized, "Accessing uninitialized config!");
    return &parameters;
  }

  /// Access the parameters.
  const Parameters& params() const {
    LOG_TRACE_IF(!initialized, "Accessing uninitialized config!");
    return parameters;
  }

  /// Deleted copy constructor.
  Config& operator=(const Config& config) = delete;

 private:
  /// Parameter storage
  Parameters parameters;

  /// This parameter must be false to write the parameters.
  bool initialized = false;
};
}  // namespace QDVO

/// Global configuration.
extern QDVO::Config config;
