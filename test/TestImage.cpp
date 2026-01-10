#include "TestFixtures.h"
#include "gtest/gtest.h"
#include <Eigen/Core>

class ImageTest : public QDVOSyntheticImageTest {};

TEST_F(ImageTest, Basic) {
  auto sourceKey = insertKeyframe(QDVO::Vector3(0, 0, 0),
                                  QDVO::SO3::exp(QDVO::Vector3(0, 0, 0)));

  QDVO::Frame& source = *(*graph.getKeyframeMap().at(sourceKey));

  Eigen::Vector2d px(40, 200);
  source.imagePyr.getImage().getImageData().setZero();
  source.imagePyr.getImage().getImageData()(static_cast<Eigen::Index>(px.y()),
                                             static_cast<Eigen::Index>(px.x())) = 500;

  cv::Mat tempMat = source.imagePyr.getImage().toOpenCVImage();
  cv::Mat cvMat;
  tempMat.convertTo(cvMat, CV_16U);
  source.updateImage(cvMat);

  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x(), px.y()))
                .value(),
            500);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() + 1, px.y()))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x(), px.y() + 1))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() - 1, px.y()))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x(), px.y() - 1))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() + 1, px.y() + 1))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() - 1, px.y() + 1))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() + 1, px.y() - 1))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() - 1, px.y() - 1))
                .value(),
            0);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() + 0.5, px.y()))
                .value(),
            250);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x(), px.y() + 0.5))
                .value(),
            250);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x() - 0.5, px.y()))
                .value(),
            250);
  EXPECT_EQ(source.imagePyr.getImage()
                .getSubPixelIntensity(QDVO::Vector2(px.x(), px.y() - 0.5))
                .value(),
            250);
}

