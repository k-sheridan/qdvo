#include <gtest/gtest.h>

#include "Estimation/SlidingWindowEstimator.h"
#include "Logging.h"
#include "TestFixtures.h"

class SlidingWindowEstimatorTest : public QDVOSyntheticImageTest {
 public:
  /// This function projects the landmarks from a sourceframe into the target
  /// keyframe and initializes a correspondence distribution for them, if they
  /// are visible.
  /// @param targetKey the keyframe which correspondence distributuions will be
  /// initialized for.
  /// @param sourceKeyframeKey The keyframe which landmarks are projected from.
  void initializeCorrespondenceDistributuionsForFrame(
      KeyframeMap::key_type targetKey,
      KeyframeMap::key_type sourceKeyframeKey) {
    auto& target = *(*graph.getKeyframeMap().at(targetKey));
    auto& targetCameraModel =
        *graph.getCameraModelMap().at(target.cameraModelKey)->first;
    auto& source = *(*graph.getKeyframeMap().at(sourceKeyframeKey));
    auto& sourceCameraModel =
        *graph.getCameraModelMap().at(source.cameraModelKey)->first;

    for (auto landmarkKey : source.landmarkKeys) {
      // Warp the patch.
      auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
      QDVO::Result<QDVO::Patch> warpedPatch;
      patchWarper->warpPatchToTargetFrame(warpedPatch, landmark, source, target,
                                          graph);

      if (warpedPatch.has_value()) {
        SPDLOG_TRACE("Warped Patch to target. sum: {}",
                     warpedPatch.value().getImageData().sum());

        // The patch should correlate with itself.
        EXPECT_GT(
            patchComparer->compare(warpedPatch.value(), warpedPatch.value()),
            POTENTIAL_CORRESPONDENCE_THRESHOLD);

        auto projectionResult =
            graph.projectLandmarkToPixel(target, source, landmark);
        if (projectionResult.has_value()) {
          target.correspondenceDistributions.push_back(
              CorrespondenceDistribution(targetCameraModel.imageWidth(),
                                         targetCameraModel.imageHeight(),
                                         radialSearchPattern));

          Eigen::Vector2i center(std::round(projectionResult.value()(0)),
                                 std::round(projectionResult.value()(1)));

          SPDLOG_TRACE("Pixel center: {}", center);

          // Compare the warped patch with the center pixel.
          auto score =
              patchComparer->compare(warpedPatch.value(), target, center);
          SPDLOG_TRACE("Center score {}", score.value());

          auto nPcs =
              target.correspondenceDistributions.back().initializeDistribution(
                  targetCameraModel, target, landmarkKey, center,
                  MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchComparer,
                  warpedPatch.value());

          SPDLOG_INFO("Potential Correspondences: {}", nPcs);
        }
      }
    }
  }

  /// Draws point landmark.
  void drawPointLandmark(KeyframeMap::key_type sourceKeyframe,
                         LandmarkMap::key_type landmarkKey,
                         KeyframeMap::key_type targetKeyframe) {
    auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
    EXPECT_EQ(sourceKeyframe, landmark.parentFrameKey);
    auto pointInSource = landmark.getEuclideanPoint();
    auto& source = *(*graph.getKeyframeMap().at(sourceKeyframe));
    auto pointInWorld = source.imustate.getSE3() * pointInSource;
    auto& target = *(*graph.getKeyframeMap().at(targetKeyframe));

    // Render the point landmark.
    auto featurePositionResult =
        renderPointLandmark(targetKeyframe, pointInWorld, 1.0);

    if (featurePositionResult.has_value()) {
      // Verify that the feature position is valid.
      auto projResult = graph.projectLandmarkToPixel(
          targetKeyframe, sourceKeyframe, landmarkKey);
      SPDLOG_TRACE("Rendered feature at: \n{}\n",
                   featurePositionResult.value());

      ASSERT_TRUE(projResult.has_value());
      EXPECT_TRUE(
          featurePositionResult.value().isApprox(projResult.value(), 1e-6));
      EXPECT_NE(target.imagePyr.getImage().getImageData()(
                    std::round(projResult.value()(1)),
                    std::round(projResult.value()(0))),
                0);

      // Verify that the drawn landmark matches.
      QDVO::Result<QDVO::Patch> warpedPatch;
      patchWarper->warpPatchToTargetFrame(warpedPatch, landmark, source, target,
                                          graph);
      EXPECT_TRUE(warpedPatch.has_value());
      SPDLOG_TRACE("Warped Patch: \n{}\n", warpedPatch.value().getImageData());
      Eigen::Vector2i center(std::round(featurePositionResult.value()(0)),
                             std::round(featurePositionResult.value()(1)));
      auto score = patchComparer->compare(warpedPatch.value(), target, center);
      EXPECT_TRUE(score.has_value());
      SPDLOG_TRACE("Match score {}", score.value());
      EXPECT_GT(score.value(), POTENTIAL_CORRESPONDENCE_THRESHOLD);
    }
  }
};

TEST_F(SlidingWindowEstimatorTest, ThreeFrameCornersOnlySolve) {
  // Create 3 frames with blank images.
  KeyframeMap::key_type sourceKey;
  sourceKey = insertKeyframe(QDVO::Vector3(0, 0, 0),
                             QDVO::SO3::exp(QDVO::Vector3(0, 0, 0)));
  KeyframeMap::key_type targetKey1;
  targetKey1 = insertKeyframe(QDVO::Vector3(0.2, 0, 0),
                              QDVO::SO3::exp(QDVO::Vector3(0, -0.02, 0)));
  KeyframeMap::key_type targetKey2;
  targetKey2 = insertKeyframe(QDVO::Vector3(0.5, 0, 0),
                              QDVO::SO3::exp(QDVO::Vector3(0, -0.04, 0)));

  // Insert landmarks into the source keyframe.
  auto l1 = insertLandmark(sourceKey, QDVO::Vector3(0, 0, 1), 0.5);
  auto l2 = insertLandmark(sourceKey, QDVO::Vector3(0.5, 0, 1), 0.5);
  auto l3 = insertLandmark(sourceKey, QDVO::Vector3(0.2, -0.1, 1), 0.5);
  auto l4 = insertLandmark(sourceKey, QDVO::Vector3(-0.5, -0.5, 1), 0.5);

  // Set all keyframes to active.
  (*graph.getKeyframeMap().at(targetKey1))->status =
      QDVO::Frame::FrameStatus::ACTIVE;
  (*graph.getKeyframeMap().at(targetKey2))->status =
      QDVO::Frame::FrameStatus::ACTIVE;
  (*graph.getKeyframeMap().at(sourceKey))->status =
      QDVO::Frame::FrameStatus::ACTIVE;

  // Render all of the landmarks.
  drawPointLandmark(sourceKey, l1, sourceKey);
  drawPointLandmark(sourceKey, l2, sourceKey);
  drawPointLandmark(sourceKey, l3, sourceKey);
  drawPointLandmark(sourceKey, l4, sourceKey);

  drawPointLandmark(sourceKey, l1, targetKey1);
  drawPointLandmark(sourceKey, l2, targetKey1);
  drawPointLandmark(sourceKey, l3, targetKey1);
  drawPointLandmark(sourceKey, l4, targetKey1);

  EXPECT_NEAR((*graph.getKeyframeMap().at(targetKey1))
                  ->imagePyr.getImage()
                  .getImageData()
                  .sum(),
              4, 1e-3);

  drawPointLandmark(sourceKey, l1, targetKey2);
  drawPointLandmark(sourceKey, l2, targetKey2);
  drawPointLandmark(sourceKey, l3, targetKey2);
  drawPointLandmark(sourceKey, l4, targetKey2);

  EXPECT_NEAR((*graph.getKeyframeMap().at(targetKey2))
                  ->imagePyr.getImage()
                  .getImageData()
                  .sum(),
              4, 1e-3);

  // Initialize correspondence distributions.
  initializeCorrespondenceDistributuionsForFrame(targetKey1, sourceKey);
  initializeCorrespondenceDistributuionsForFrame(targetKey2, sourceKey);

  // Run the sliding window estimator.
  QDVO::SlidingWindowEstimator swe;

  swe.run(graph);

  // 8 error terms should have been created.
  EXPECT_EQ(swe.errorTermContainer.getErrorTermMap<QDVO::QuasiDirectErrorTerm>()
                .size(),
            8);
  // There should be 4 inverse depth variables.
  EXPECT_EQ(swe.variableContainer.getVariableMap<ArgMin::InverseDepth>().size(),
            4);
  // There should be 3 se3 variables.
  EXPECT_EQ(swe.variableContainer.getVariableMap<ArgMin::SE3>().size(), 3);

  // Verify that the values match up to some scale parameter.a
  // Use the first landmark to determine the scaling factor.
}
