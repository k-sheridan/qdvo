#pragma once

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/complex.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>

#include "DataStructures/Frame.h"
#include "DataStructures/Graph.h"
#include "DataStructures/IMUState.h"
#include "DataStructures/Landmark.h"
#include "Estimation/FrontFndVisualOdometry.h"
#include "Estimation/SlidingWindowEstimator.h"
#include "Optimizer/PSDSchurSolver.h"

namespace cereal {

template <class Archive, class Variable>
void serialize(Archive& ar, ArgMin::TypedSlotMapKey<Variable>& key) {
  ar& cereal::make_nvp("index", key.index);
  ar& cereal::make_nvp("generation", key.generation);
}

template <class Archive, typename ValueType, typename KeyType>
void serialize(Archive& ar, ArgMin::SlotMap<ValueType, KeyType>& map) {
  for (auto it = map.begin(); it != map.end(); it++) {
    auto key = map.getKeyFromDataIndex(std::distance(map.begin(), it));
    ar& cereal::make_nvp(
        std::to_string(key.index) + "-" + std::to_string(key.generation),
        *(it));
  }
}

/// Specialization of the slotmap serialization function which removes the
/// unique_ptr.
template <class Archive, typename ValueType, typename KeyType>
void serialize(Archive& ar,
               ArgMin::SlotMap<std::unique_ptr<ValueType>, KeyType>& map) {
  for (auto it = map.begin(); it != map.end(); it++) {
    auto key = map.getKeyFromDataIndex(std::distance(map.begin(), it));
    ar& cereal::make_nvp(
        std::to_string(key.index) + "-" + std::to_string(key.generation),
        *(*it));
  }
}

template <class Archive, int Rows, int Cols, typename Scalar>
void serialize(Archive& ar, Eigen::Matrix<Scalar, Rows, Cols>& matrix) {
  ar& cereal::make_nvp("rows", matrix.rows());
  ar& cereal::make_nvp("cols", matrix.cols());
  std::vector<Scalar> data(matrix.data(),
                           matrix.data() + matrix.rows() * matrix.cols());
  ar& cereal::make_nvp("data", data);
}

template <class Archive>
void serialize(Archive& ar, QDVO::EpipolarDepthEstimator& depthEstimator) {
  ar& cereal::make_nvp("initialized", depthEstimator.initialized);
  ar& cereal::make_nvp("error", depthEstimator.error);
  ar& cereal::make_nvp("attempts", depthEstimator.attempts);
}

template <class Archive>
void serialize(Archive& ar, QDVO::Landmark& landmark) {
  ar& cereal::make_nvp("status", landmark.status);
  ar& cereal::make_nvp("parent_frame_key", landmark.parentFrameKey);
  ar& cereal::make_nvp("inverse_depth", landmark.dinv);
  ar& cereal::make_nvp("bearing", landmark.bearing);
  ar& cereal::make_nvp("px", landmark.px);
  ar& cereal::make_nvp("depth_estimator", landmark.depthEstimator);
}

template <class Archive>
void serialize(Archive& ar, QDVO::SO3& so3) {
  ar& cereal::make_nvp("w", so3.unit_quaternion().w());
  ar& cereal::make_nvp("x", so3.unit_quaternion().x());
  ar& cereal::make_nvp("y", so3.unit_quaternion().y());
  ar& cereal::make_nvp("z", so3.unit_quaternion().z());
}

template <class Archive>
void serialize(Archive& ar, QDVO::SE3& se3) {
  ar& cereal::make_nvp("so3", se3.so3());
  ar& cereal::make_nvp("pos", se3.translation());
}

template <class Archive>
void serialize(Archive& ar, QDVO::IMUState& imustate) {
  ar& cereal::make_nvp("time_ns", std::to_string(imustate.time));
  ar& cereal::make_nvp("pos", imustate.pos);
  ar& cereal::make_nvp("vel", imustate.vel);
  ar& cereal::make_nvp("attitude", imustate.attitude);
}

template <class Archive>
void serialize(Archive& ar,
               QDVO::CorrespondenceDistribution& correspondenceDistribution) {
  ar& cereal::make_nvp("initialized", correspondenceDistribution.initialized);
  ar& cereal::make_nvp("warped_patch",
                       correspondenceDistribution.warpedPatch.getImageData());
  ar& cereal::make_nvp("landmark_key", correspondenceDistribution.landmarkKey);
}

template <class Archive>
void serialize(Archive& ar, QDVO::Frame& frame) {
  ar& cereal::make_nvp("camera_model_key", frame.cameraModelKey);
  ar& cereal::make_nvp("extrinsic_key", frame.extrinsicKey);
  ar& cereal::make_nvp("imustate", frame.imustate);
  ar& cereal::make_nvp("correspondence_distributions",
                       frame.correspondenceDistributions);
}

template <class Archive>
void serialize(Archive& ar, QDVO::Graph& graph) {
  ar& cereal::make_nvp("current_frame_key", graph.getCurrentFrameKey());
  ar& cereal::make_nvp("extrinsics", graph.getExtrinsicMap());
  ar& cereal::make_nvp("keyframes", graph.getKeyframeMap());
  ar& cereal::make_nvp("landmarks", graph.getLandmarkMap());
}

template <class Archive>
void serialize(Archive& ar, QDVO::FrontEndVisualOdometry::Solver::SolveResult& result) {
  ar& cereal::make_nvp("iteration_sse", result.whitenedSqError);
}

template <class Archive>
void serialize(Archive& ar, QDVO::SlidingWindowEstimator::Solver::SolveResult& result) {
  ar& cereal::make_nvp("iteration_sse", result.whitenedSqError);
}

template <class Archive>
void serialize(Archive& ar, QDVO::SlidingWindowEstimator& swe) {
  std::vector<QDVO::KeyframeMap::key_type> frameKeys;
  for (auto it = swe.poseKeyMap.begin(); it != swe.poseKeyMap.end(); it++) {
    frameKeys.push_back(swe.poseKeyMap.getKeyFromDataIndex(
        std::distance(swe.poseKeyMap.begin(), it)));
  }
  ar& cereal::make_nvp("frame_keys_in_window", frameKeys);

  ar& cereal::make_nvp("solve_result", swe.lastSolveResult);
}

template <class Archive>
void serialize(Archive& ar, QDVO::FrontEndVisualOdometry& fevo) {
  ar& cereal::make_nvp("frame_key_solved_for", fevo.lastSolvedFrameKey);
  ar& cereal::make_nvp("solve_result", fevo.lastSolveResult);
}

}  // namespace cereal
