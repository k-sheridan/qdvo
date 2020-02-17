#include "FeatureDetector.h"
#include "TestFixtures.h"
#include "gtest/gtest.h"
#include "DataStructures/Feature.h"

class FeatureDetectorTest : public QDVOSyntheticImageTest {};

TEST_F(FeatureDetectorTest, Basic) {
  auto sourceKey = insertKeyframe(QDVO::Vector3(0, 0, 0),
                                  QDVO::SO3::exp(QDVO::Vector3(0, 0, 0)));

  QDVO::FeatureDetector featureDetector;

  QDVO::Frame& source = *(*graph.getKeyframeMap().at(sourceKey));
  auto results = featureDetector.detectFeatures(source);

  EXPECT_EQ(results.size(), 0);

  auto featurePositionResult =
      renderPointLandmark(sourceKey, QDVO::Vector3(0, 0, 1), 500);
}

