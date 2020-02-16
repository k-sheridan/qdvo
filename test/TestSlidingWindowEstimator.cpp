#include <gtest/gtest.h>

#include "Estimation/SlidingWindowEstimator.h"
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
        std::cout << "Warped Patch to target. " << std::endl;
        auto projectionResult =
            graph.projectLandmarkToPixel(target, source, landmark);
        if (projectionResult.has_value()) {
          target.correspondenceDistributions.push_back(
              CorrespondenceDistribution(targetCameraModel.imageWidth(),
                                         targetCameraModel.imageHeight(),
                                         radialSearchPattern));

          Eigen::Vector2i center(std::round(projectionResult.value()(0)),
                                 std::round(projectionResult.value()(1)));

          std::cout << "Pixel center: " << center << std::endl;

          auto nPcs =
              target.correspondenceDistributions.back().initializeDistribution(
                  targetCameraModel, target, landmarkKey, center,
                  MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchComparer,
                  warpedPatch.value());

	  std::cout << "Potential Correspondences: " << nPcs << std::endl;
        }
      }
    }
  }

  /// Inserts a keyframe with a blank image.
  KeyframeMap::key_type insertKeyframe(QDVO::Vector3 pos, QDVO::SO3 attitude) {
    auto sourceKeyframeKey = insertFrame();
    auto& sourceKeyframe = *(*graph.getKeyframeMap().at(sourceKeyframeKey));
    insertBlankImageIntoKeyframe(sourceKeyframeKey);
    sourceKeyframe.imustate.pos = pos;
    sourceKeyframe.imustate.attitude = attitude;
    return sourceKeyframeKey;
  }

  /// Draws point landmark.
  void drawPointLandmark(KeyframeMap::key_type sourceKeyframe,
                         LandmarkMap::key_type landmarkKey,
                         KeyframeMap::key_type targetKeyframe) {
    auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
    EXPECT_EQ(sourceKeyframe, landmark.parentFrameKey);
    auto pointInSource = landmark.getEuclideanPoint();
    auto& source = *(*graph.getKeyframeMap().at(sourceKeyframe));
    auto pointInWorld = source.imustate.getSE3().inverse() * pointInSource;

    // Render the point landmark.
    auto featurePositionResult =
        renderPointLandmark(targetKeyframe, pointInWorld);

    if (featurePositionResult.has_value()) {
      // Verify that the feature position is valid.
      auto projResult = graph.projectLandmarkToPixel(
          targetKeyframe, sourceKeyframe, landmarkKey);
      ASSERT_TRUE(projResult.has_value());
      EXPECT_TRUE(
          featurePositionResult.value().isApprox(projResult.value(), 1e-6));
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
  auto l3 = insertLandmark(sourceKey, QDVO::Vector3(0, 0.5, 1), 0.5);
  auto l4 = insertLandmark(sourceKey, QDVO::Vector3(-0.5, -0.5, 1), 0.5);

  // Render all of the landmarks.
  drawPointLandmark(sourceKey, l1, sourceKey);
  drawPointLandmark(sourceKey, l2, sourceKey);
  drawPointLandmark(sourceKey, l3, sourceKey);
  drawPointLandmark(sourceKey, l4, sourceKey);

  drawPointLandmark(sourceKey, l1, targetKey1);
  drawPointLandmark(sourceKey, l2, targetKey1);
  drawPointLandmark(sourceKey, l3, targetKey1);
  drawPointLandmark(sourceKey, l4, targetKey1);

  drawPointLandmark(sourceKey, l1, targetKey2);
  drawPointLandmark(sourceKey, l2, targetKey2);
  drawPointLandmark(sourceKey, l3, targetKey2);
  drawPointLandmark(sourceKey, l4, targetKey2);

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
}
