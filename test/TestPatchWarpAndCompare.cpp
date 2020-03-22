#include "Logging.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "TestFixtures.h"
#include "gtest/gtest.h"

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
        renderPointLandmark(targetKeyframe, pointInWorld, 500);

    EXPECT_TRUE(featurePositionResult.has_value());
    return featurePositionResult.value();
  }

  void testBasic(QDVO::Vector2 px_source, double dinv) {
    // Create two keyframes at the same location.
    auto sourceKey =
        insertKeyframe(QDVO::Vector3(0.2, -0.1, 0.5),
                       QDVO::SO3::exp(QDVO::Vector3(0.2, -0.1, 0.5)));
    auto targetKey =
        insertKeyframe(QDVO::Vector3(0.2, -0.1, 0.5),
                       QDVO::SO3::exp(QDVO::Vector3(0.2, -0.1, 0.5)));

    auto& source = *(*graph.getKeyframeMap().at(sourceKey));
    auto bearing = graph.getCameraModelMap()
                       .at(source.cameraModelKey)
                       ->first->unproject(px_source)
                       .value();

    // Insert a landmark into the source keyframe.
    auto lKey = insertLandmark(sourceKey, bearing, dinv);

    // Render two identical point landmarks in both frames.
    auto px1 = drawPointLandmark(sourceKey, lKey, targetKey);
    auto referencePixel =
        graph.projectLandmarkToPixel(targetKey, sourceKey, lKey);
    EXPECT_TRUE(referencePixel.has_value());
    EXPECT_NEAR(referencePixel.value()(0), px1(0), 1e-8);
    EXPECT_NEAR(referencePixel.value()(1), px1(1), 1e-8);
    drawPointLandmark(sourceKey, lKey, sourceKey);

    auto& target = *(*graph.getKeyframeMap().at(targetKey));
    auto& landmark = *graph.getLandmarkMap().at(lKey);

    // Warp A patch in to the source and target frame.
    QDVO::Result<QDVO::Patch> patch1;
    patchWarper->warpPatchToTargetFrame(patch1, landmark, source, target,
                                        graph);
    EXPECT_TRUE(patch1.has_value());
    EXPECT_EQ(patch1.value().getImageData().sum(), 500);

    LOG_TRACE("patch1: \n{}\n", patch1->getImageData());
    QDVO::Result<QDVO::Patch> patch2;
    patchWarper->warpPatchToTargetFrame(patch2, landmark, source, source,
                                        graph);
    EXPECT_TRUE(patch2.has_value());
    EXPECT_EQ(patch2.value().getImageData().sum(), 500);

    // Compare the two patches expecting that the score is above the threshhold.
    auto score = patchComparer->compare(patch1.value(), patch2.value());
    EXPECT_TRUE(score.has_value());
    EXPECT_GT(score.value(), POTENTIAL_CORRESPONDENCE_THRESHOLD);

    // Remove the keyframes and landmark from the graph.
    graph.getKeyframeMap().erase(sourceKey);
    graph.getKeyframeMap().erase(targetKey);
    graph.getLandmarkMap().erase(lKey);
  }

  void testPerspectiveChange(QDVO::Vector2 px_source, double dinv,
                             QDVO::Vector3 sourcePos, QDVO::Vector3 sourceSo3,
                             QDVO::Vector3 targetPos, QDVO::Vector3 targetSo3) {
    // Create two keyframes at the same location.
    auto sourceKey = insertKeyframe(sourcePos, QDVO::SO3::exp(sourceSo3));
    auto targetKey = insertKeyframe(targetPos, QDVO::SO3::exp(targetSo3));

    auto& source = *(*graph.getKeyframeMap().at(sourceKey));
    auto bearing = graph.getCameraModelMap()
                       .at(source.cameraModelKey)
                       ->first->unproject(px_source)
                       .value();

    // Insert a landmark into the source keyframe.
    auto lKey = insertLandmark(sourceKey, bearing, dinv);

    // Render two identical point landmarks in both frames.
    auto px1 = drawPointLandmark(sourceKey, lKey, targetKey);
    auto referencePixel =
        graph.projectLandmarkToPixel(targetKey, sourceKey, lKey);
    EXPECT_TRUE(referencePixel.has_value());
    EXPECT_NEAR(referencePixel.value()(0), px1(0), 1e-8);
    EXPECT_NEAR(referencePixel.value()(1), px1(1), 1e-8);
    drawPointLandmark(sourceKey, lKey, sourceKey);

    auto& target = *(*graph.getKeyframeMap().at(targetKey));
    auto& landmark = *graph.getLandmarkMap().at(lKey);

    // Warp A patch in to the source and target frame.
    QDVO::Result<QDVO::Patch> patch1;
    patchWarper->warpPatchToTargetFrame(patch1, landmark, source, target,
                                        graph);
    EXPECT_TRUE(patch1.has_value());

    LOG_TRACE("patch1: \n{}\n", patch1->getImageData());
    QDVO::Result<QDVO::Patch> patch2;
    patchWarper->warpPatchToTargetFrame(patch2, landmark, source, source,
                                        graph);
    EXPECT_TRUE(patch2.has_value());

    // Compare the two patches expecting that the score is above the threshhold.
    auto score = patchComparer->compare(patch1.value(), patch2.value());
    EXPECT_TRUE(score.has_value());
    EXPECT_GT(score.value(), POTENTIAL_CORRESPONDENCE_THRESHOLD);
    LOG_TRACE("Score of patches: {}", score.value());

    // Remove the keyframes and landmark from the graph.
    graph.getKeyframeMap().erase(sourceKey);
    graph.getKeyframeMap().erase(targetKey);
    graph.getLandmarkMap().erase(lKey);
  }

  /// Draws point landmark.
  QDVO::Vector2 drawEdgeLandmark(KeyframeMap::key_type sourceKeyframe,
                                 LandmarkMap::key_type landmarkKey,
                                 KeyframeMap::key_type targetKeyframe,
                                 QDVO::Vector3 normalInSource) {
    auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
    EXPECT_EQ(sourceKeyframe, landmark.parentFrameKey);
    auto pointInSource = landmark.getEuclideanPoint();
    auto& source = *(*graph.getKeyframeMap().at(sourceKeyframe));
    auto pointInWorld = source.imustate.getSE3() * pointInSource;
    auto& target = *(*graph.getKeyframeMap().at(targetKeyframe));

    auto normalInWorld = source.imustate.getSE3().inverse() * normalInSource;
    // Render the point landmark.
    auto featurePositionResult = renderEdgeLandmark(
        targetKeyframe, pointInWorld, normalInWorld, 1, 0.01, 500);

    EXPECT_TRUE(featurePositionResult.has_value());
    return featurePositionResult.value();
  }

  void testEdgePerspectiveChange(QDVO::Vector2 px_source, double dinv,
                                 QDVO::Vector3 sourcePos,
                                 QDVO::Vector3 sourceSo3,
                                 QDVO::Vector3 targetPos,
                                 QDVO::Vector3 targetSo3,
                                 QDVO::Vector3 normalInSource) {
    // Create two keyframes at the same location.
    auto sourceKey = insertKeyframe(sourcePos, QDVO::SO3::exp(sourceSo3));
    auto targetKey = insertKeyframe(targetPos, QDVO::SO3::exp(targetSo3));

    auto& source = *(*graph.getKeyframeMap().at(sourceKey));
    auto bearing = graph.getCameraModelMap()
                       .at(source.cameraModelKey)
                       ->first->unproject(px_source)
                       .value();

    // Insert a landmark into the source keyframe.
    auto lKey = insertLandmark(sourceKey, bearing, dinv);

    // Render two identical point landmarks in both frames.
    auto px1 = drawEdgeLandmark(sourceKey, lKey, targetKey, normalInSource);
    auto referencePixel =
        graph.projectLandmarkToPixel(targetKey, sourceKey, lKey);
    EXPECT_TRUE(referencePixel.has_value());
    EXPECT_NEAR(referencePixel.value()(0), px1(0), 1e-8);
    EXPECT_NEAR(referencePixel.value()(1), px1(1), 1e-8);
    drawPointLandmark(sourceKey, lKey, sourceKey);

    auto& target = *(*graph.getKeyframeMap().at(targetKey));
    auto& landmark = *graph.getLandmarkMap().at(lKey);

    // Warp A patch in to the source and target frame.
    QDVO::Result<QDVO::Patch> patch1;
    patchWarper->warpPatchToTargetFrame(patch1, landmark, source, target,
                                        graph);
    EXPECT_TRUE(patch1.has_value());

    LOG_TRACE("patch1: \n{}\n", patch1->getImageData());
    QDVO::Result<QDVO::Patch> patch2;
    patchWarper->warpPatchToTargetFrame(patch2, landmark, source, source,
                                        graph);
    EXPECT_TRUE(patch2.has_value());

    // Compare the two patches expecting that the score is above the threshhold.
    auto score = patchComparer->compare(patch1.value(), patch2.value());
    EXPECT_TRUE(score.has_value());
    EXPECT_GT(score.value(), POTENTIAL_CORRESPONDENCE_THRESHOLD);
    LOG_TRACE("Score of patches: {}", score.value());

    // Remove the keyframes and landmark from the graph.
    graph.getKeyframeMap().erase(sourceKey);
    graph.getKeyframeMap().erase(targetKey);
    graph.getLandmarkMap().erase(lKey);
  }
};

TEST_F(PatchWarpAndCompareTest, Basic) {
  testBasic(QDVO::Vector2(255, 255), 0.5);
  testBasic(QDVO::Vector2(400, 100), 0.5);
  testBasic(QDVO::Vector2(450, 450), 0.5);
  testBasic(QDVO::Vector2(255, 100), 0.5);
  testBasic(QDVO::Vector2(100, 50), 0.5);
  testBasic(QDVO::Vector2(50, 450), 0.5);
  testBasic(QDVO::Vector2(300, 256), 0.5);
}

TEST_F(PatchWarpAndCompareTest, PerspectiveChange) {
  testPerspectiveChange(QDVO::Vector2(255, 255), 0.5, QDVO::Vector3(0, 0, 0),
                        QDVO::Vector3(0, 0, 0), QDVO::Vector3(1, 0, 0),
                        QDVO::Vector3(-0.1, 0, 0));
  testPerspectiveChange(QDVO::Vector2(240, 255), 0.5, QDVO::Vector3(0, 0, 0),
                        QDVO::Vector3(0, 0, 0), QDVO::Vector3(0, 1, 0),
                        QDVO::Vector3(0, 0.1, 0));
  testPerspectiveChange(QDVO::Vector2(255, 200), 0.5, QDVO::Vector3(0, 0, 1),
                        QDVO::Vector3(0, 0, 0), QDVO::Vector3(1, -0.1, 0),
                        QDVO::Vector3(0, 0, 0.1));
  testPerspectiveChange(QDVO::Vector2(255, 400), 0.5, QDVO::Vector3(1, 2, 0),
                        QDVO::Vector3(0.1, 0, 0), QDVO::Vector3(1, 2, 0),
                        QDVO::Vector3(0, 0, 0));
}

TEST_F(PatchWarpAndCompareTest, EdgePerspectiveChange) {
  testEdgePerspectiveChange(QDVO::Vector2(255, 255), 0.5,
                            QDVO::Vector3(0, 0, 0), QDVO::Vector3(0, 0, 0),
                            QDVO::Vector3(1, 0, 0), QDVO::Vector3(-0.1, 0, 0),
                            QDVO::Vector3(0, 1, 0));
  testEdgePerspectiveChange(QDVO::Vector2(240, 255), 0.5,
                            QDVO::Vector3(0, 0, 0), QDVO::Vector3(0, 0, 0),
                            QDVO::Vector3(0, 1, 0), QDVO::Vector3(0, 0.1, 0),
                            QDVO::Vector3(0, 1, 0));
  testEdgePerspectiveChange(QDVO::Vector2(255, 200), 0.5,
                            QDVO::Vector3(0, 0, 1), QDVO::Vector3(0, 0, 0),
                            QDVO::Vector3(1, -0.1, 0), QDVO::Vector3(0, 0, 0.1),
                            QDVO::Vector3(0, 1, 0));
  testEdgePerspectiveChange(QDVO::Vector2(255, 400), 0.5,
                            QDVO::Vector3(1, 2, 0), QDVO::Vector3(0.1, 0, 0),
                            QDVO::Vector3(1, 2, 0), QDVO::Vector3(0, 0, 0),
                            QDVO::Vector3(0, 1, 0));
}
