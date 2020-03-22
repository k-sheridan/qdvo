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
        LOG_TRACE("Warped Patch to target. sum: {}",
                  warpedPatch.value().getImageData().sum());

        // The patch should correlate with itself.
        EXPECT_GT(
            patchComparer->compare(warpedPatch.value(), warpedPatch.value()),
            POTENTIAL_CORRESPONDENCE_THRESHOLD);

        auto projectionResult =
            graph.projectLandmarkToPixel(target, source, landmark);
        if (projectionResult.has_value()) {
          target.correspondenceDistributions.insert(CorrespondenceDistribution(
              targetCameraModel.imageWidth(), targetCameraModel.imageHeight(),
              radialSearchPattern));

          Eigen::Vector2i center(std::round(projectionResult.value()(0)),
                                 std::round(projectionResult.value()(1)));

          // Compare the warped patch with the center pixel.
          auto score =
              patchComparer->compare(warpedPatch.value(), target, center);
          LOG_TRACE("Center score {} for landmark: {}", score.value(),
                    landmarkKey.index);

          auto nPcs = (target.correspondenceDistributions.end() - 1)
                          ->initializeDistribution(
                              targetCameraModel, target, landmarkKey, center,
                              MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS,
                              patchComparer, warpedPatch.value());

          LOG_TRACE("Pixel center: {}", center);

          LOG_TRACE("Correspondence Distribution around center pixel: \n{}\n",
                    target.correspondenceDistributions.back().extractScores(
                        center, Eigen::Vector2i(15, 15)));

          LOG_TRACE("Potential Correspondences: {}", nPcs);
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
      LOG_TRACE("Rendered feature at: \n{}\n", featurePositionResult.value());
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
      LOG_TRACE("Warped Patch: \n{}\n", warpedPatch.value().getImageData());
      Eigen::Vector2i center(std::round(featurePositionResult.value()(0)),
                             std::round(featurePositionResult.value()(1)));
      auto score = patchComparer->compare(warpedPatch.value(), target, center);
      EXPECT_TRUE(score.has_value());
      LOG_TRACE("Match score {}", score.value());
      if (score.value() < POTENTIAL_CORRESPONDENCE_THRESHOLD) {
        LOG_WARN(
            "Score low for expected feature. Score: {} \n WarpedPatch: \n{}\n, "
            "TargetPatch: \n{}\n",
            score.value(), warpedPatch.value().getImageData(),
            target.imagePyr.getImage()
                .getImageData()
                .block<PATCH_WIDTH, PATCH_WIDTH>(
                    std::round(featurePositionResult.value().y()) -
                        PATCH_RADIUS,
                    std::round(featurePositionResult.value().x()) -
                        PATCH_RADIUS));
      }
    }
  }
};

using V3 = Eigen::Vector3d;

struct Params {
  V3 pos1;
  V3 pos2;
  V3 pos3;

  V3 so31;
  V3 so32;
  V3 so33;

  V3 b1;
  V3 b2;
  V3 b3;
  V3 b4;

  double d1;
  double d2;
  double d3;
  double d4;

  friend std::ostream& operator<<(std::ostream& os, Params const& a) {
    return os << a.d1 << ", " << a.d2 << ", " << a.d3 << ", " << a.d4 << ", "
              << '\n';
  }
};

/// Set up a parameterized test.
/// Variables: {sourcePos,sourceSo3, targetPos1,targetSo31,
/// targetPos2,targetSo32, dinv1, dinv2, dinv3, dinv4}
class SWEParamTest : public SlidingWindowEstimatorTest,
                     public ::testing::WithParamInterface<Params> {};

TEST_P(SWEParamTest, ThreeFrameCornersOnlySolve) {
  // Get the params.
  const Params& p = GetParam();

  // Create 3 frames with blank images.
  KeyframeMap::key_type sourceKey;
  sourceKey = insertKeyframe(p.pos1, QDVO::SO3::exp(p.so31));
  KeyframeMap::key_type targetKey1;
  targetKey1 = insertKeyframe(p.pos2, QDVO::SO3::exp(p.so32));
  KeyframeMap::key_type targetKey2;
  targetKey2 = insertKeyframe(p.pos3, QDVO::SO3::exp(p.so33));

  // Insert landmarks into the source keyframe.
  auto l1 = insertLandmark(sourceKey, p.b1, p.d1);
  auto l2 = insertLandmark(sourceKey, p.b2, p.d2);
  auto l3 = insertLandmark(sourceKey, p.b3, p.d3);
  auto l4 = insertLandmark(sourceKey, p.b4, p.d4);

  std::vector<QDVO::KeyframeMap::key_type> keyframes = {sourceKey, targetKey1,
                                                        targetKey2};
  auto computePixelPositions =
      [](QDVO::Graph& graph, std::vector<QDVO::KeyframeMap::key_type> keyframes)
      -> std::vector<QDVO::Vector2> {
    std::vector<QDVO::Vector2> results;
    for (auto keyframeKey : keyframes) {
      for (auto lKey :
           (*graph.getKeyframeMap().at(keyframeKey))->landmarkKeys) {
        LOG_TRACE("compute pixel for landmark {}", lKey.index);
        for (auto targetKey : keyframes) {
          auto result =
              graph.projectLandmarkToPixel(targetKey, keyframeKey, lKey);
          if (result.has_value()) {
            results.push_back(result.value());
          }
        }
      }
    }
    return results;
  };

  auto gtPixels = computePixelPositions(graph, keyframes);

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

  // Test the reprojection errors.
  auto estPixels = computePixelPositions(graph, keyframes);
  EXPECT_EQ(gtPixels.size(), estPixels.size());

  for (int i = 0; i < gtPixels.size(); ++i) {
    LOG_TRACE("Pixel error: {}", (estPixels.at(i) - gtPixels.at(i)).norm());
    EXPECT_NEAR((estPixels.at(i) - gtPixels.at(i)).norm(), 0, 1);
  }
}

// Set up the test parameters.
auto p1 = V3(0, 0, 0);
auto p2 = V3(0.2, 0, 0);
auto p3 = V3(0.2, 0.4, 0);
auto p4 = V3(0, -0.4, 0);
auto p5 = V3(-0.1, -0.2, 0);

auto s1 = V3(0.01, 0, 0.02);
auto s2 = V3(0, 0.02, -0.02);
auto s3 = V3(0.02, 0.02, 0);

auto d1 = 0.5;
auto d2 = 0.1;
auto d3 = 0.2;
auto d4 = 0.01;

auto b1 = V3(0.1, -0.1, 1);
auto b2 = V3(0.5, 0, 1);
auto b3 = V3(-0.1, -0.4, 1);
auto b4 = V3(0.1, 0.1, 1);
auto b5 = V3(0, 0, 1);

//clang-format off
std::vector<Params> cases = {
    {p1, p2, p3, s1, s2, s3, b1, b2, b3, b4, d1, d1, d1, d1},
    {p1, p4, p3, s1, s2, s2, b1, b2, b3, b4, d2, d1, d1, d2},
    {p1, p4, p3, s1, s2, s2, b1, b4, b3, b5, d2, d1, d1, d2},
    {p1, p2, p3, s1, s2, s3, b1, b2, b3, b4, d1, d1, d3, d1},
    {p5, p4, p3, s2, s2, s3, b1, b2, b3, b4, d3, d3, d3, d2},
    {p1, p4, p2, s1, s3, s3, b1, b2, b3, b4, d1, d1, d1, d3},
    {p1, p3, p2, s1, s3, s3, b1, b2, b3, b4, d1, d4, d1, d4},
    {p1, p2, p3, s1, s2, s3, b1, b2, b3, b4, d1, d4, d1, d1}};
//clang-format on

INSTANTIATE_TEST_SUITE_P(ParameterizedSWETestGroup, SWEParamTest,
                         ::testing::ValuesIn(cases));
