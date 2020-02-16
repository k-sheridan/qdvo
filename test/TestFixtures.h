#pragma once

#include <gtest/gtest.h>

#include "CameraModel.hpp"
#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Frame.h"
#include "DataStructures/Graph.h"
#include "DataStructures/Image.h"
#include "EquidistantCameraModel.h"
#include "GlobalDefinitions.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "RadialSearchPattern.h"
#include "Types.h"

using namespace QDVO;

class QDVOBasicTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  QDVO::CameraModelMap::key_type addCameraToGraph(
      std::unique_ptr<QDVO::CameraModel> cameraModel) {
    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                   Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    return graph.insertCamera(std::move(cameraModel), unit);
  }

  std::unique_ptr<QDVO::EquidistantCameraModel> cm =
      std::make_unique<QDVO::EquidistantCameraModel>(
          300, 301, 255, 256, PI / 2.1, 512, 400,
          Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257,
                          -0.0020532361418706202, 0.00020293673591811182));

  std::shared_ptr<QDVO::PatchComparer> patchComparer =
      std::make_shared<QDVO::PatchComparer>(QDVO::PatchComparer());

  std::shared_ptr<QDVO::PatchWarper> patchWarper;

  std::shared_ptr<const RadialSearchPattern> radialSearchPattern = std::make_shared<const RadialSearchPattern>(40);

  QDVO::Graph graph;
};

/**
 * This fixture contains some helper functions which
 * make it easy to add landmarks and frames to the graph.
 */
class QDVOSimpleGraphTest : public QDVOBasicTest {
 protected:
  void SetUp() override {
    // Insert a camera model into the graph.
    cameraModelKey = addCameraToGraph(std::make_unique<CameraModel>(*cm));

    // Create an extrinsic for the keyframes.
    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                   Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    extrinsicKey = graph.getExtrinsicMap().insert(unit);
  }

  /// Inserts and setsup a landmark in the desired keyframe.
  LandmarkMap::key_type insertLandmark(KeyframeMap::key_type hostFrameKey,
                                       QDVO::Vector3 bearing, double dinv) {
    auto& sourceKeyframe = *(*graph.getKeyframeMap().at(hostFrameKey));

    auto landmarkKey = graph.getLandmarkMap().insert(QDVO::Landmark());
    auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
    landmark.parentFrameKey = hostFrameKey;
    sourceKeyframe.landmarkKeys.push_back(landmarkKey);

    // Set the landmark bearing and dinv.
    landmark.bearing = QDVO::Vector3(0, 0, 1);
    landmark.dinv = dinv;

    // Compute the source pixel position.
    auto sourcePixelResult =
        graph.projectLandmarkToPixel(sourceKeyframe, sourceKeyframe, landmark);
    EXPECT_TRUE(sourcePixelResult.has_value());

    // Set the source pixel position.
    landmark.px = sourcePixelResult.value();

    return landmarkKey;
  }

  KeyframeMap::key_type insertFrame(
      QDVO::IMUState imustate = QDVO::IMUState(),
      std::unique_ptr<QDVO::Frame> frame = std::make_unique<QDVO::Frame>()) {
    // Create the target keyframe.
    auto targetKeyframeKey = graph.getKeyframeMap().insert(std::move(frame));
    auto& targetKeyframe = *(*graph.getKeyframeMap().at(targetKeyframeKey));

    targetKeyframe.imustate = imustate;

    // Set the target keyframe's extrinsic key.
    targetKeyframe.extrinsicKey = extrinsicKey;

    // Set the target frame's camera model.
    targetKeyframe.cameraModelKey = cameraModelKey;

    return targetKeyframeKey;
  }

  CameraModelMap::key_type cameraModelKey;
  ExtrinsicMap::key_type extrinsicKey;
};

class QDVOSyntheticImageTest : public QDVOSimpleGraphTest {
 protected:
  /// Initializes the keyframe's image with zero.
  void insertBlankImageIntoKeyframe(KeyframeMap::key_type keyframeKey) {
    auto& sourceKeyframe = *(*graph.getKeyframeMap().at(keyframeKey));
    // Create the source image.
    QDVO::Image image;
    // TODO don't hard code the image size.
    image.getImageData() = Eigen::MatrixXf(512, 512);
    image.getImageData().setZero();

    // Update the source keyframe's image.
    cv::Mat cvMat;
    image.toOpenCVImage().convertTo(cvMat, CV_16U);
    sourceKeyframe.updateImage(cvMat);
  }

  /// Renders a point landmark into a keyframes' image.
  /// @param keyframeKey the key to the keyframe which the point will be
  /// rendered into.
  /// @param pointInWorld the position of the point in the world frame.
  /// @param landmarkIntensity the projected brightness of the landmark.
  /// @return optional pixel position of landmark in keyframe.
  QDVO::Result<QDVO::Vector2> renderPointLandmark(
      KeyframeMap::key_type keyframeKey, QDVO::Vector3 pointInWorld,
      double landmarkIntensity = 5.0) {
    // project the point into the keyframe.
    auto& keyframe = *(*graph.getKeyframeMap().at(keyframeKey));
    auto pointInKeyframe = keyframe.imustate.getSE3().inverse() * pointInWorld;

    auto& cameraModel =
        graph.getCameraModelMap().at(keyframe.cameraModelKey)->first;

    auto projectionResult = cameraModel->project(pointInKeyframe);

    // Check if the point is visible in this frame.
    if (projectionResult.has_value()) {
      // Get the image and insert a bright pixel at the projected point.
      auto& image = keyframe.imagePyr.getImage().getImageData();
      image(std::round(projectionResult.value()(0)),
            std::round(projectionResult.value()(1))) = landmarkIntensity;
    }

    // Return the projection result.
    return projectionResult;
  }

  /// Renders an edge landmark into a keyframes' image.
  /// @param keyframeKey the key to the keyframe which the point will be
  /// rendered into.
  /// @param pointInWorld the position of the point in the world frame.
  /// @param normalInWorld the direction of the edge landmark in the world.
  /// @param length The length in meters of the landmark in the world.
  /// @param sampleResolution Distance between sample points on edge in meters.
  /// @param landmarkIntensity the projected brightness of the landmark.
  /// @return optional pixel position of landmark in keyframe.
  QDVO::Result<QDVO::Vector2> renderEdgeLandmark(
      KeyframeMap::key_type keyframeKey, QDVO::Vector3 pointInWorld,
      QDVO::Vector3 normalInWorld, double length, double sampleResolution,
      double landmarkIntensity = 5.0) {
    // project the point into the keyframe.
    auto& keyframe = *(*graph.getKeyframeMap().at(keyframeKey));
    auto pointInKeyframe = keyframe.imustate.getSE3().inverse() * pointInWorld;
    auto normalInKeyframe =
        keyframe.imustate.attitude.inverse() * normalInWorld;

    auto& cameraModel =
        graph.getCameraModelMap().at(keyframe.cameraModelKey)->first;

    auto centerProjectionResult = cameraModel->project(pointInKeyframe);

    for (double scale = -(length / 2); scale < (length / 2);
         scale += sampleResolution) {
      auto projectionResult =
          cameraModel->project(pointInKeyframe + normalInKeyframe * scale);

      // Check if the point is visible in this frame.
      if (projectionResult.has_value()) {
        // Get the image and insert a bright pixel at the projected point.
        auto& image = keyframe.imagePyr.getImage().getImageData();
        image(std::round(projectionResult.value()(0)),
              std::round(projectionResult.value()(1))) = landmarkIntensity;
      }
    }

    // Return the projection result.
    return centerProjectionResult;
  }
};
