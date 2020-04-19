#pragma once

#include "DataStructures/Graph.h"
#include "Serialization.h"

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

    template <class Archive>
    void serialize(Archive& ar) {
      ar& cereal::make_nvp("garbage", graph.getCurrentFrameKey().generation);
    }
  };
  /// Initialize the tracking log.
  TrackingLog(std::ofstream& os, std::string datasetName) : archive(os) {
    archive& cereal::make_nvp("dataset_name", datasetName);
    archive.setNextName("frames");
    archive.startNode();
  }

  /// Serialize the tracking state.
  void logTrackingState(Graph& graph) {
    TrackingState ts = {graph};
    archive& cereal::make_nvp(std::to_string(frameNumber), ts);
    ++frameNumber;
  }

  /// Archive used to save tracking states.
  cereal::JSONOutputArchive archive;
  /// Frame counter.
  int frameNumber = 0;
};
}  // namespace QDVO
