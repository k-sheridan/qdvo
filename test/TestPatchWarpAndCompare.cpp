#include "PatchComparer.h"
#include "PatchWarper.h"
#include "TestFixtures.h"
#include "gtest/gtest.h"
#include "Logging.h"

class PatchWarpAndCompareTest : public QDVOSyntheticImageTest {
 public:
  /// Draws point landmark.
  QDVO::Vector2 drawPointLandmark(KeyframeMap::key_type sourceKeyframe,
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

    EXPECT_TRUE(featurePositionResult.has_value());
    EXPECT_EQ(target.imagePyr.getImage().getImageData()(
                  std::round(featurePositionResult.value()(1)),
                  std::round(featurePositionResult.value()(0))),
              1.0);
    return featurePositionResult.value();
  }

  void testBasic(QDVO::Vector3 bearing, double dinv) {
    // Create two keyframes at the same location.
    auto sourceKey = insertKeyframe(QDVO::Vector3(0.2, 0, 0),
                                    QDVO::SO3::exp(QDVO::Vector3(0, -0.02, 0)));
    auto targetKey = insertKeyframe(QDVO::Vector3(0.2, 0, 0),
                                    QDVO::SO3::exp(QDVO::Vector3(0, -0.02, 0)));

    // Insert a landmark into the source keyframe.
    auto lKey = insertLandmark(sourceKey, bearing, dinv);

    // Render two identical point landmarks in both frames.
    auto px1 = drawPointLandmark(sourceKey, lKey, targetKey);
    auto referencePixel =
        graph.projectLandmarkToPixel(targetKey, sourceKey, lKey);
    EXPECT_TRUE(referencePixel.has_value());
    EXPECT_EQ(referencePixel.value()(0), px1(0));
    EXPECT_EQ(referencePixel.value()(1), px1(1));
    drawPointLandmark(sourceKey, lKey, sourceKey);

    auto& source = *(*graph.getKeyframeMap().at(sourceKey));
    auto& target = *(*graph.getKeyframeMap().at(targetKey));
    auto& landmark = *graph.getLandmarkMap().at(lKey);

    // Warp A patch in to the source and target frame.
    QDVO::Result<QDVO::Patch> patch1;
    patchWarper->warpPatchToTargetFrame(patch1, landmark, source, target,
                                        graph);
    EXPECT_TRUE(patch1.has_value());
    EXPECT_EQ(patch1.value().getImageData().sum(), 1);
    EXPECT_EQ(patch1.value().getImageData()(PATCH_RADIUS, PATCH_RADIUS), 1);

    SPDLOG_TRACE("patch1: \n{}\n", patch1->getImageData());
    QDVO::Result<QDVO::Patch> patch2;
    patchWarper->warpPatchToTargetFrame(patch2, landmark, source, source,
                                        graph);
    EXPECT_TRUE(patch2.has_value());
    EXPECT_EQ(patch2.value().getImageData().sum(), 1);
    EXPECT_EQ(patch2.value().getImageData()(PATCH_RADIUS, PATCH_RADIUS), 1);

    // Compare the two patches expecting that the score is above the threshhold.
    auto score = patchComparer->compare(patch1.value(), patch2.value());
    EXPECT_TRUE(score.has_value());
    EXPECT_GT(score.value(), POTENTIAL_CORRESPONDENCE_THRESHOLD);

    // Remove the keyframes and landmark from the graph.
    graph.getKeyframeMap().erase(sourceKey);
    graph.getKeyframeMap().erase(targetKey);
    graph.getLandmarkMap().erase(lKey);
  }
};

TEST_F(PatchWarpAndCompareTest, Basic) {
  testBasic(QDVO::Vector3(0, 0, 1), 0.5);
  testBasic(QDVO::Vector3(0.5, 0, 1), 0.5);
  testBasic(QDVO::Vector3(0, -0.5, 1), 0.5);
}

