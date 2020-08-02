#include "TestFixtures.h"
#include "gtest/gtest.h"

TEST_F(QDVOSimpleGraphTest, TestProjectionHelpers) {
  auto sourceKeyframeKey = insertFrame();
  auto targetKeyframeKey = insertFrame();
  auto landmarkKey =
      insertLandmark(sourceKeyframeKey, QDVO::Vector3(0, 0, 1), 1);

  auto projResult = graph.projectLandmarkToPixel(
      targetKeyframeKey, sourceKeyframeKey, landmarkKey);

  EXPECT_TRUE(projResult.has_value());
  EXPECT_EQ(projResult.value()(0), cm->cx);
  EXPECT_EQ(projResult.value()(1), cm->cy);

  auto& sourceKeyframe = *(*graph.getKeyframeMap().at(sourceKeyframeKey));
  auto& targetKeyframe = *(*graph.getKeyframeMap().at(targetKeyframeKey));
  auto& landmark = *graph.getLandmarkMap().at(landmarkKey);

  sourceKeyframe.imustate.pos = QDVO::Vector3(-0.2, -0.1, 0.1);
  sourceKeyframe.imustate.attitude =
      QDVO::SO3::exp(QDVO::Vector3(0.01, 0.02, -0.01));
  targetKeyframe.imustate.pos = QDVO::Vector3(0.1, 0.1, 0.1);
  targetKeyframe.imustate.attitude =
      QDVO::SO3::exp(QDVO::Vector3(-0.01, 0.01, -0.02));

  auto keyframeTransform =
      graph.computeRelativeKeyframeTransform(sourceKeyframe, targetKeyframe);

  auto extrinsic = *graph.getExtrinsicMap().at(extrinsicKey);
  auto referenceTransform =
      (sourceKeyframe.imustate.getSE3() * extrinsic).inverse() *
      (targetKeyframe.imustate.getSE3() * extrinsic);

  EXPECT_TRUE(
      referenceTransform.matrix().isApprox(keyframeTransform.matrix(), 1e-6));

  EXPECT_NEAR(graph.computeAverageSceneDepthInFrame(targetKeyframe),
              (keyframeTransform * QDVO::Vector3(0, 0, 1)).z(), 1e-6);
}
