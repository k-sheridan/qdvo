#pragma once

#include <memory>

#include "GlobalDefinitions.h"
#include "Logging.h"

namespace QDVO {
class Config {
 public:
  struct Parameters {
    /// global settings file
    int nKeyframes = 8;

    /// Allow for certain functions to be executed in parallel.
    bool allowParallelExecution = true;

    /// from the DSO keyframe selection criteria.
    double weightAvgPixelFlow = 0.04;
    double weightAvgTranslationalFlow = 0.12;

    /// Settings for the sliding window estimator.
    double huberWidth = 1;
    double pixelOutlierThreshold = 3;

    bool fitGaussian = false;

    /// The image pyramid level used for FEVO.
    int coarseImageCorrespondenceLevel = 2;

    /// feature detection settings
    struct FeatureDetectionSettings {
      int nFeaturesDesired = 400;
      int nSections = 20;
      double harrisK = 0.05;
      double edgeWeight = 0.1;
      double invariantThreshold = 0.1;
      double minimumNormalizedGradientMagnitude = 0.0306;
      double featureSeparation = 5;
    } feature_detection;

    /// Epipolar depth estimator settings.
    struct EpipolarDepthEstimatorSettings {
      int maximumAttempts = 5000;
      double maximumErrorPerDepth = 0.1;
      int maximumHypotheses = 20;
      double minimumDepth = 0.4;
      double maximumDepth = 20;
      double resolution = 0.5;
      int landmarksPerThread = 50;
      bool enableThreadingCostModel = true;
    } epipolar_depth_estimator;

    /// Thresholds which determine lost tracking.
    struct LostTrackingSettings {
      int visibleFeatureThreshold = 10;
      double sqErrorThreshold = 100;
    } lost_tracking_settings;

    struct MarginalizationSettings {
      double minimumLandmarkRatio = 0.001;
      double maximumLandmarkRatio = 0.15;
    } marginalization_settings;

    struct ActivationSettings {
      double depthErrorBias = 0.75;
    } activation_settings;
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
    // LOG_TRACE_IF(!initialized, "Accessing uninitialized config!");
    return &parameters;
  }

  /// Access the parameters.
  const Parameters& params() const {
    // LOG_TRACE_IF(!initialized, "Accessing uninitialized config!");
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
