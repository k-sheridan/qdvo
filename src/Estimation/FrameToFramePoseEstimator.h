#pragma once

#include "DataStructures/Frame.h"
#include "DataStructures/Graph.h"
#include "Logging.h"
#include "Optimizer/ErrorTermBase.h"
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
    const QDVO::Graph &graph;
    const KeyframeMap::key_type frame1;
    const KeyframeMap::key_type frame2;
    /// originalPixel = imageScaleRatio * scaledPixel
    double imageScaleRatio;
    const QDVO::Image &scaledImage1;
    const QDVO::Image &scaledImage2;
    QDVO::ImageIntensityType image1Mean;
    QDVO::ImageIntensityType image2Mean;
    QDVO::ImageIntensityType image1StandardDeviation;
    QDVO::ImageIntensityType image2StandardDeviation;
  };

  DirectSE2ErrorTerm(const SharedData &sharedData,
                     Eigen::Matrix<int, 2, 1> frame1ScaledPixel)
      : sharedData(sharedData) {
    const auto &cameraModel =
        sharedData.graph.getCameraModelForKeyframe(sharedData.frame1);

    frame1Intensity = sharedData.scaledImage1.at(frame1ScaledPixel);

    auto unprojectedPixel = cameraModel.unproject(
        frame1ScaledPixel.cast<double>() * sharedData.imageScaleRatio);

    CHECK(unprojectedPixel.has_value(), "Failed to unproject pixel.");
    frame1UnitBearing = unprojectedPixel.value().normalized();
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

    const auto &cameraModel =
        sharedData.graph.getCameraModelForKeyframe(sharedData.frame2);

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
      auto &jac = (std::get<0>(variableJacobians));

      auto scaledDProj = dProj / sharedData.imageScaleRatio;

      Eigen::Matrix<double, 2, 3> dPi;
      dPi << 1 / pt(2, 0), 0, -pt(0, 0) / (pt(2, 0) * pt(2, 0)), 0,
          1 / pt(2, 0), -pt(1, 0) / (pt(2, 0) * pt(2, 0));

      int row = std::round(scaledPixel.y());
      int col = std::round(scaledPixel.x());

      if (row <= 0 || col <= 0 || row >= sharedData.scaledImage2.rows() - 2 ||
          col >= sharedData.scaledImage2.cols() - 2) {
        linearizationValid = false;
        return;
      }

      auto patch = sharedData.scaledImage2.getImageData().block<3, 3>(row - 1, col - 1);

      /// dI/dx image gradient kernel.
      const Eigen::Matrix<QDVO::ImageIntensityType, 3, 3> Gx =
          ((Eigen::Matrix<QDVO::ImageIntensityType, 3, 3>() << -1, 0, 1, -2, 0,
            2, -1, 0, 1)
               .finished());
      /// dI/dy image gradient kernel.
      const Eigen::Matrix<QDVO::ImageIntensityType, 3, 3> Gy =
          ((Eigen::Matrix<QDVO::ImageIntensityType, 3, 3>() << 1, 2, 1, 0, 0, 0,
            -1, -2, -1)
               .finished());

      Eigen::Matrix<double, 1, 2> dI_dxdy((Gx.array() * patch.array()).sum(),
                                          (Gy.array() * patch.array()).sum());

      jac = -dI_dxdy * dPi * R.value.matrix() * Sophus::SO3d::hat(frame1UnitBearing);

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
  bool estimateRelativePose(const QDVO::Graph &graph,
                            KeyframeMap::key_type frame1,
                            KeyframeMap::key_type frame2,
                            QDVO::SE3 &T_frame1_frame2) {
    return false;
  }
};
}  // namespace QDVO
