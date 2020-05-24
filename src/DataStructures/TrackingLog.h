#pragma once

#include "DataStructures/Graph.h"
#include "Serialization.h"
#include "BasicPipeline.h"

namespace QDVO {
/**
 * The tracking log is meant to store a log of the tracking state at each frame.
 *
 * The tracking state should include enough information to produce metrics and
 * do analysis on the tracking performance on a frame by frame basis.
 *
 * Each tracking state should include:
 * \verbatim embed:rst
 * - Current frame imustate
 * - Keyframe imustates
 * - Correspondence distribution potential correspondences (pixel, score).
 * - Correspondence warped patches (store as array + dimensions)
 * - Correspondence distribution initialized bool.
 * - Landmarks (dinv, bearing, status, parentFrameKey)
 * - Front end visual odometry result (residuals, number of error terms).
 * - Sliding window estimation result (residuals, number of error terms, number
 * of iterations).
 * - All keys should be saved with with a generation to eliminate ambiguity.
 * \endverbatim
 */
struct TrackingLog {
 public:
  struct TrackingState {
    QDVO::Graph& graph;
    QDVO::SlidingWindowEstimator& swe;
    QDVO::FrontEndVisualOdometry fevo;
    QDVO::BasicPipeline::Status trackingStatus;

    template <class Archive>
    void serialize(Archive& ar) {
      ar& cereal::make_nvp("tracking_status", trackingStatus);
      ar& cereal::make_nvp("graph", graph);
      ar& cereal::make_nvp("sliding_window_estimator", swe);
      ar& cereal::make_nvp("front_end_visual_odometry", fevo);
    }
  };
  /// Initialize the tracking log.
  TrackingLog(std::ofstream& os, std::string datasetName,
              cereal::JSONOutputArchive::Options options =
                  cereal::JSONOutputArchive::Options(
                      6, cereal::JSONOutputArchive::Options::IndentChar::space,
                      0))
      : archive(os, options) {
    archive& cereal::make_nvp("dataset_name", datasetName);
    // Manually start the frames node.
    archive.setNextName("frames");
    archive.startNode();
  }

  /// Serialize the tracking state.
  void logTrackingState(Graph& graph, SlidingWindowEstimator& swe,
                        FrontEndVisualOdometry& fevo, BasicPipeline::Status status) {
    TrackingState ts = {graph, swe, fevo, status};
    archive& cereal::make_nvp(std::to_string(frameNumber), ts);
    ++frameNumber;
  }

  ~TrackingLog() {
    archive.finishNode();
  }

  /// Archive used to save tracking states.
  cereal::JSONOutputArchive archive;
  /// Frame counter.
  int frameNumber = 0;
};
}  // namespace QDVO
