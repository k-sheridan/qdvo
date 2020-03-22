#pragma once

#include "CameraModel.hpp"
#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Graph.h"
#include "Optimizer/ErrorTermBase.h"
#include "Optimizer/Variables/InverseDepth.h"
#include "Optimizer/Variables/SE3.h"
#include "Types.h"

namespace QDVO {

class Graph;

class QuasiDirectErrorTerm : public ArgMin::ErrorTermBase<
                                 ArgMin::Scalar<double>, ArgMin::Dimension<2>,
                                 ArgMin::VariableGroup<ArgMin::SE3, ArgMin::SE3,
                                                       ArgMin::InverseDepth>> {
 public:
  /// The information matrix representing the gaussian uncertainty in this error
  /// term.
  Eigen::Matrix<double, 2, 2> information;
  /// A cached bearing used to project the landmark.
  Eigen::Vector2d bearing;
  /// A pointer to the correspondence distribution in the target frame used to
  /// evaluate this error term.
  CorrespondenceDistributionMap::key_type correspondenceDistributionKey;
  /// A pointer to the graph containing the landmarks and correspondence
  /// distributions.
  Graph* graph = nullptr;
  /// The keys to the target and host Frame in the graph.
  KeyframeMap::key_type targetFrameKey, hostFrameKey;
  /// The key to the landmark.
  LandmarkMap::key_type landmarkKey;

  QuasiDirectErrorTerm(
      ArgMin::VariableKey<ArgMin::SE3> hostFrameVariableKey,
      ArgMin::VariableKey<ArgMin::SE3> targetFrameVariableKey,
      ArgMin::VariableKey<ArgMin::InverseDepth> dinvVariableKey,
      Eigen::Matrix<double, 2, 2> information,
      CorrespondenceDistributionMap::key_type cdKey, Graph* graph,
      KeyframeMap::key_type hostFrameKey, KeyframeMap::key_type targetFrameKey,
      LandmarkMap::key_type landmarkKey) {
    std::get<0>(variableKeys) = hostFrameVariableKey;
    std::get<1>(variableKeys) = targetFrameVariableKey;
    std::get<2>(variableKeys) = dinvVariableKey;
    this->information = information;
    correspondenceDistributionKey = cdKey;
    this->graph = graph;
    this->hostFrameKey = hostFrameKey;
    this->targetFrameKey = targetFrameKey;
    this->landmarkKey = landmarkKey;
  }

  template <typename... Variables>
  void evaluate(ArgMin::VariableContainer<Variables...>& variables,
                bool relinearize = false) {
    Sophus::SE3d& host = std::get<0>(variablePointers)->value;
    Sophus::SE3d& target = std::get<1>(variablePointers)->value;
    double& inverseDepth = std::get<2>(variablePointers)->value;

    assert(graph != nullptr);
    Frame& targetFrame = *(*graph->getKeyframeMap().at(targetFrameKey));
    Frame& sourceFrame = *(*graph->getKeyframeMap().at(hostFrameKey));
    Landmark& landmark = *graph->getLandmarkMap().at(landmarkKey);

    assert(landmark.parentFrameKey == hostFrameKey);

    auto& cm = graph->getCameraModelMap().at(targetFrame.cameraModelKey)->first;
    auto& correspondenceDistribution =
        *targetFrame.correspondenceDistributions.at(
            correspondenceDistributionKey);
    Eigen::Vector3d bearing = landmark.bearing;

    // Compute the residual.
    auto pointInHost =
        Eigen::Vector3d(bearing(0, 0) / inverseDepth,
                        bearing(1, 0) / inverseDepth, 1 / inverseDepth);
    Eigen::Vector3d pointInTarget = target.inverse() * host * pointInHost;

    // Project the point into a pixel.
    Eigen::Matrix<double, 2, 2> projJac;
    auto projectionResult = cm->project(pointInTarget, &projJac);

    // Check if the projection failed.
    if (!projectionResult.has_value()) {
      linearizationValid = false;
      return;
    }

    auto residualResult = correspondenceDistribution.computeResidual(
        *cm, targetFrame, projectionResult.value());

    if (!residualResult.has_value()) {
      linearizationValid = false;
      return;
    }

    residual = residualResult.value();

    if (relinearize) {
      Eigen::Matrix<double, 2, 6>& hostJacobian =
          (std::get<0>(variableJacobians));
      Eigen::Matrix<double, 2, 6>& targetJacobian =
          (std::get<1>(variableJacobians));
      Eigen::Matrix<double, 2, 1>& dinvJacobian =
          (std::get<2>(variableJacobians));

      Eigen::Matrix<double, 2, 3> dPi;
      dPi << 1 / pointInTarget(2, 0), 0,
          -pointInTarget(0, 0) / (pointInTarget(2, 0) * pointInTarget(2, 0)), 0,
          1 / pointInTarget(2, 0),
          -pointInTarget(1, 0) / (pointInTarget(2, 0) * pointInTarget(2, 0));

      // Compute the jacobians.
      auto A = target.so3().matrix();
      auto B = host.so3().matrix();
      // T =
      // graph.extrinsics.getImu2CameraTransform(graph.FrameContainer{observationFrameIdx}.camID);
      // C = T(1:3, 1:3);

      auto& d = target.translation();
      auto& e = host.translation();
      // f = T(1:3, 4);

      // dinv =
      // graph.FrameContainer{landmarkFrameIdx}.landmarks{landmarkIdx}.dinv; u0
      // =
      // [graph.FrameContainer{landmarkFrameIdx}.landmarks{landmarkIdx}.bearing;
      // 1];

      // J_dinv = -projJac * dPi * (C'*A'*B*C*u0*1/dinv^2);
      dinvJacobian =
          projJac * dPi * (A.transpose() * B * pointInHost * 1 / inverseDepth);

      // J_dphi_o = projJac * dPi * (C' * so3Hat(A'*(B*C*u0*(1/dinv) + B*f + e -
      // d)));
      targetJacobian.block(0, 0, 2, 3) =
          -projJac * dPi *
          (Sophus::SO3d::hat(A.transpose() * (B * pointInHost + e - d)));
      // J_dt_o = -projJac * dPi * (C'*A');
      targetJacobian.block(0, 3, 2, 3) = projJac * dPi * (A.transpose());

      // J_dphi_o = -projJac * dPi * C'*A'*B * so3Hat(C*u0/dinv + f);
      hostJacobian.block(0, 0, 2, 3) =
          projJac * dPi * A.transpose() * B * Sophus::SO3d::hat(pointInHost);
      // J_dt_o = projJac * dPi * (C'*A');
      hostJacobian.block(0, 3, 2, 3) = -projJac * dPi * (A.transpose());

      linearizationValid = true;
    } else {
      linearizationValid = false;
    }
  }
};

}  // namespace QDVO
