#include "TestFixtures.h"
#include "gtest/gtest.h"

class ImageTest : public QDVOSyntheticImageTest {};

TEST_F(ImageTest, Basic) {
  auto sourceKey = insertKeyframe(QDVO::Vector3(0, 0, 0),
                                  QDVO::SO3::exp(QDVO::Vector3(0, 0, 0)));

  QDVO::Frame& source = *(*graph.getKeyframeMap().at(sourceKey));

  auto featurePositionResult =
      renderPointLandmark(sourceKey, QDVO::Vector3(0, 0, 1), 500);

  cv::Mat cvMat;
  source.imagePyr.getImage().toOpenCVImage().convertTo(cvMat, CV_16U);
  source.updateImage(cvMat);

  EXPECT_EQ(source.imagePyr.getImage().getImageData()(cm->cy, cm->cx), 500);

  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx, cm->cy)).value(),
            500);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx + 1, cm->cy)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx, cm->cy + 1)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx - 1, cm->cy)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx, cm->cy - 1)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx + 1, cm->cy + 1)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx - 1, cm->cy + 1)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx + 1, cm->cy - 1)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx - 1, cm->cy - 1)).value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx + 0.5, cm->cy)).value(),
            250);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx, cm->cy + 0.5)).value(),
            250);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx - 0.5, cm->cy)).value(),
            250);
  EXPECT_EQ(source.imagePyr.getImage().getSubPixelIntensity(
                QDVO::Vector2(cm->cx, cm->cy - 0.5)).value(),
            250);
}

