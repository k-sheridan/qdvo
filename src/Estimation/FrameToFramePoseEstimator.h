#pragma once

#include "DataStructures/Frame.h"
#include "DataStructures/Graph.h"
#include "Logging.h"
#include "Optimizer/ErrorTermBase.h"
#include "Optimizer/NumericalDifferentiator.h"
#include "Optimizer/PSDSchurSolver.h"
#include "Optimizer/Variables/SO3.h"
#include "Types.h"

namespace QDVO {
/// Error term which photometrically aligns two images.
class DirectSE2ErrorTerm
    : public ArgMin::ErrorTermBase<ArgMin::Scalar<double>, ArgMin::Dimension<1>,
                                   ArgMin::VariableGroup<ArgMin::SO3>> {
 public:
  struct SharedData {
    QDVO::Graph &graph;
    const KeyframeMap::key_type frame1;
    const KeyframeMap::key_type frame2;
    /// originalPixel = imageScaleRatio * scaledPixel
    double imageScaleRatio;
    const QDVO::Image &scaledImage1;
    const QDVO::Image &scaledImage2;
  };

  DirectSE2ErrorTerm(const SharedData &sharedData,
                     Eigen::Matrix<int, 2, 1> frame1ScaledPixel,
                     ArgMin::VariableKey<ArgMin::SO3> key)
      : sharedData(sharedData) {
    const QDVO::Frame &keyframe =
        *(*sharedData.graph.getKeyframeMap().at(sharedData.frame1));
    const auto &cameraModel = *(sharedData.graph.getCameraModelMap()
                                    .at(keyframe.cameraModelKey)
                                    ->first);

    frame1Intensity = sharedData.scaledImage1.at(frame1ScaledPixel);

    auto unprojectedPixel = cameraModel.unproject(
        frame1ScaledPixel.cast<double>() * sharedData.imageScaleRatio);

    CHECK(unprojectedPixel.has_value(), "Failed to unproject pixel.");
    frame1UnitBearing = unprojectedPixel.value().normalized();
    std::get<0>(variableKeys) = key;
  }

  /// Data which is shared between error terms.
  const SharedData &sharedData;

  QDVO::ImageIntensityType frame1Intensity;
  Eigen::Matrix<double, 3, 1> frame1UnitBearing;

  template <typename... Variables>
  void evaluate(ArgMin::VariableContainer<Variables...> &variables,
                bool relinearize) {
    auto &R = *(std::get<0>(variablePointers));

    // I(dx) = I(Pi(R * Exp(dx) * b))
    // E(dx) = z - I(Pi(R * Exp(dx) * b))
    // E(dx) ~= z - I(Pi(R * (I + hat(dx)) * b))
    // dE(dx)/dx ~= z - I(Pi(R * hat(dx) * b))
    // dE(dx)/dx = -dI * dPi * R * hat(b)

    const QDVO::Frame &keyframe =
        *(*sharedData.graph.getKeyframeMap().at(sharedData.frame2));
    const auto &cameraModel = *(sharedData.graph.getCameraModelMap()
                                    .at(keyframe.cameraModelKey)
                                    ->first);

    auto pt = R.value * frame1UnitBearing;
    Eigen::Matrix<double, 2, 2> dProj;
    auto px = cameraModel.project(pt, &dProj);
    if (!px.has_value()) {
      linearizationValid = false;
      return;
    }

    auto scaledPixel = px.value() / sharedData.imageScaleRatio;
    auto intensity = sharedData.scaledImage2.getSubPixelIntensity(scaledPixel);

    if (!intensity.has_value()) {
      linearizationValid = false;
      return;
    }

    residual(0, 0) = frame1Intensity - intensity.value();

    if (relinearize) {
      // auto &jac = (std::get<0>(variableJacobians));

      // auto scaledDProj = dProj / sharedData.imageScaleRatio;

      // Eigen::Matrix<double, 2, 3> dPi;
      // dPi << 1 / pt(2, 0), 0, -pt(0, 0) / (pt(2, 0) * pt(2, 0)), 0,
      //    1 / pt(2, 0), -pt(1, 0) / (pt(2, 0) * pt(2, 0));

      // int row = std::round(scaledPixel.y());
      // int col = std::round(scaledPixel.x());

      // if (row <= 0 || col <= 0 || row >= sharedData.scaledImage2.rows() - 2
      // ||
      //    col >= sharedData.scaledImage2.cols() - 2) {
      //  linearizationValid = false;
      //  return;
      //}

      // auto patch =
      //    sharedData.scaledImage2.getImageData().block<3, 3>(row - 1, col -
      //    1);

      // LOG_INFO("patch: {}", patch);

      ///// dI/dx image gradient kernel.
      // const Eigen::Matrix<QDVO::ImageIntensityType, 3, 3> Gx =
      //    ((Eigen::Matrix<QDVO::ImageIntensityType, 3, 3>() << -1, 0, 1, -2,
      //    0,
      //      2, -1, 0, 1)
      //         .finished());
      ///// dI/dy image gradient kernel.
      // const Eigen::Matrix<QDVO::ImageIntensityType, 3, 3> Gy =
      //    ((Eigen::Matrix<QDVO::ImageIntensityType, 3, 3>() << 1, 2, 1, 0, 0,
      //    0,
      //      -1, -2, -1)
      //         .finished());

      // Eigen::Matrix<double, 1, 2> dI_dxdy((Gx.array() * patch.array()).sum(),
      //                                    (Gy.array() * patch.array()).sum());

      // LOG_INFO("array: {}", dI_dxdy);

      // jac = -dI_dxdy * dPi * R.value.matrix() *
      //      Sophus::SO3d::hat(frame1UnitBearing);

      variableJacobians = numericallyDifferentiate(*this, variables);

      linearizationValid = true;
    } else {
      linearizationValid = false;
    }

    information.setIdentity();
  }
};

class FrameToFramePoseEstimator {
 public:
  struct Settings {
    /// Number of pixels between the sample points.
    int pixelSeparation = 1;
  } settings;

  FrameToFramePoseEstimator() {}

  /**
   * Attempt to estimate T_frame1_frame2
   * @return Was the estimation successful?
   */
  bool estimateRelativePose(QDVO::Graph &graph, KeyframeMap::key_type frame1,
                            KeyframeMap::key_type frame2,
                            QDVO::SE3 &T_frame1_frame2,
                            QDVO::SE3 initialEstimate = QDVO::SE3()) {
    int level = (*graph.getKeyframeMap().at(frame1))->imagePyr.levels() - 1;
    double ratio = std::pow(2, level);
    // Copy the images.
    QDVO::Image znImage1 =
        (*graph.getKeyframeMap().at(frame1))->imagePyr.getImage(level);
    QDVO::Image znImage2 =
        (*graph.getKeyframeMap().at(frame2))->imagePyr.getImage(level);

    QDVO::ImageIntensityType image1Mean =
        znImage1.getImageData().sum() / (znImage1.rows() * znImage1.cols());
    QDVO::ImageIntensityType image2Mean =
        znImage2.getImageData().sum() / (znImage2.rows() * znImage2.cols());

    znImage1.getImageData().array() -= image1Mean;
    znImage2.getImageData().array() -= image2Mean;

    QDVO::ImageIntensityType image1Sd =
        znImage1.getImageData().cwiseAbs2().sum() /
        (znImage1.rows() * znImage1.cols());
    QDVO::ImageIntensityType image2Sd =
        znImage2.getImageData().cwiseAbs2().sum() /
        (znImage2.rows() * znImage2.cols());

    if (image1Sd <= 1e-8 || image2Sd <= 1e-8) {
      LOG_ERROR("Image has no texture.");
      return false;
    }

    znImage1.getImageData() *= (1.0 / image1Sd);
    znImage2.getImageData() *= (1.0 / image2Sd);

    LOG_INFO("Normalized highest level ({}) image.", level);

    /// Variable container which stores the pose refined by the optimizer.
    ArgMin::VariableContainer<ArgMin::SO3> variableContainer;

    /// Stores all error terms used during the optimization.
    ArgMin::ErrorTermContainer<DirectSE2ErrorTerm> errorTermContainer;

    /// A prior used for the solve.
    ArgMin::GaussianPrior<ArgMin::Scalar<double>,
                          ArgMin::VariableGroup<ArgMin::SO3>>
        prior;

    using LossFunction = ArgMin::HuberLossFunction<double>;

    LossFunction lossFunction = LossFunction(100);

    using Solver = ArgMin::PSDSchurSolver<
        ArgMin::Scalar<double>, ArgMin::LossFunction<LossFunction>,
        ArgMin::ErrorTermGroup<DirectSE2ErrorTerm>,
        ArgMin::VariableGroup<ArgMin::SO3>, ArgMin::VariableGroup<>>;

    /// Solver used to refine the pose.
    Solver solver = Solver(lossFunction);

    DirectSE2ErrorTerm::SharedData sharedData = {graph, frame1,   frame2,
                                                 ratio, znImage1, znImage2};

    ArgMin::SO3 R;
    R.value = initialEstimate.so3();
    auto variableKey = variableContainer.insert(R);

    for (int x = settings.pixelSeparation;
         x < znImage1.cols() - settings.pixelSeparation; ++x) {
      for (int y = settings.pixelSeparation;
           y < znImage1.rows() - settings.pixelSeparation; ++y) {
        DirectSE2ErrorTerm et(sharedData, {x, y}, variableKey);
        errorTermContainer.insert(et);
      }
    }

    auto result = solver.solveLevenbergMarquardt(variableContainer,
                                                 errorTermContainer, prior);

    LOG_INFO("Initial error: {} -> final error: {} iterations: {}",
             result.whitenedSqError.front(), result.whitenedSqError.back(),
             result.whitenedSqError.size());

    T_frame1_frame2.so3() = variableContainer.at(variableKey).value;
    T_frame1_frame2.translation() = {0, 0, 0};

    return true;
  }
};
}  // namespace QDVO
