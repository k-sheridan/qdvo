#include <gtest/gtest.h>

#include "CameraModel.hpp"
#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Frame.h"
#include "DataStructures/Image.h"
#include "EquidistantCameraModel.h"
#include "GlobalDefinitions.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "Types.h"

class CorrespondenceDistributionTest : public ::testing::Test {
 protected:
  std::unique_ptr<QDVO::EquidistantCameraModel> cm =
      std::make_unique<QDVO::EquidistantCameraModel>(
          300, 301, 255, 256, PI / 2.1, 512, 400,
          Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257,
                          -0.0020532361418706202, 0.00020293673591811182));

  std::shared_ptr<QDVO::PatchComparer> patchComparer =
      std::make_shared<QDVO::PatchComparer>(QDVO::PatchComparer());

  std::shared_ptr<QDVO::PatchWarper> patchWarper;
};

TEST_F(CorrespondenceDistributionTest, Basic) {
  std::shared_ptr<QDVO::RadialSearchPattern> rsp(
      new QDVO::RadialSearchPattern(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS));
  std::shared_ptr<QDVO::PatchComparer> patchComp(new QDVO::PatchComparer());
  QDVO::CorrespondenceDistribution dist(512, 512, rsp);

  QDVO::Image image;
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();

  // Set a single pixel in the center to high.
  image.getImageData()(256, 256) = 1.0;

  // Get a template patch from the image.
  auto patchResult =
      image.getSubPixelPatch(QDVO::Vector2(256, 256), PATCH_WIDTH);

  ASSERT_TRUE(patchResult.has_value());

  QDVO::Frame f;
  cv::Mat cvMat;
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  f.updateImage(cvMat);

  dist.initializeDistribution(*cm, f, QDVO::LandmarkMap::key_type(),
                              Eigen::Vector2i(256, 256), 25, patchComparer,
                              patchResult.value());

  // Run a search. We expect that there is one potential correspondence.
  auto searchResult = dist.search(*cm, f, Eigen::Vector2i(256, 256), 25, true);
  EXPECT_EQ(searchResult.front()->score, 1.0);

  // Compute the residual using the correspondence distribution.
  auto residual = dist.computeResidual(*cm, f, QDVO::Vector2(256, 256));
  EXPECT_TRUE(residual.has_value());
  EXPECT_EQ(residual.value().norm(), 0);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 + 10, 256));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 10, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256, 256 + 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 5, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 + 5, 256 + 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), std::sqrt(50), 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 - 5, 256 + 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), std::sqrt(50), 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 + 5, 256 - 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), std::sqrt(50), 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 - 5, 256 - 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), std::sqrt(50), 1e-6);
}

TEST_F(CorrespondenceDistributionTest, DISABLED_EdgeFeature) {
  std::shared_ptr<QDVO::RadialSearchPattern> rsp(
      new QDVO::RadialSearchPattern(MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS));
  std::shared_ptr<QDVO::PatchComparer> patchComp(new QDVO::PatchComparer());
  QDVO::CorrespondenceDistribution dist(512, 512, rsp);

  QDVO::Image image;
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();

  // Make a line in the center.
  image.getImageData().block(256, 0, 1, 512).setConstant(1);

  // Get a template patch from the image.
  auto patchResult =
      image.getSubPixelPatch(QDVO::Vector2(256, 256), PATCH_WIDTH);

  ASSERT_TRUE(patchResult.has_value());

  QDVO::Frame f;
  cv::Mat cvMat;
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  f.updateImage(cvMat);

  dist.initializeDistribution(*cm, f, QDVO::LandmarkMap::key_type(),
                              Eigen::Vector2i(256, 256), 25, patchComparer,
                              patchResult.value());

  // Run a search. We expect that there is one potential correspondence.
  auto searchResult = dist.search(*cm, f, Eigen::Vector2i(256, 256), 25, true);
  EXPECT_EQ(searchResult.front()->score, 1.0);

  // Compute the residual using the correspondence distribution.
  auto residual = dist.computeResidual(*cm, f, QDVO::Vector2(256, 256));
  EXPECT_TRUE(residual.has_value());
  EXPECT_EQ(residual.value().norm(), 0);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 + 10, 256));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 0, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256, 256 + 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 5, 1e-6);
  EXPECT_NEAR(residual.value()(1), -5, 1e-6);
  EXPECT_NEAR(residual.value()(0), 0, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 + 5, 256 + 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 5, 1e-6);
  EXPECT_NEAR(residual.value()(1), -5, 1e-6);
  EXPECT_NEAR(residual.value()(0), 0, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 - 5, 256 + 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 5, 1e-6);
  EXPECT_NEAR(residual.value()(1), -5, 1e-6);
  EXPECT_NEAR(residual.value()(0), 0, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 + 5, 256 - 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 5, 1e-6);
  EXPECT_NEAR(residual.value()(1), 5, 1e-6);
  EXPECT_NEAR(residual.value()(0), 0, 1e-6);

  // Now compute the residual from a different point.
  residual = dist.computeResidual(*cm, f, QDVO::Vector2(256 - 5, 256 - 5));
  EXPECT_TRUE(residual.has_value());
  EXPECT_NEAR(residual.value().norm(), 5, 1e-6);
  EXPECT_NEAR(residual.value()(1), 5, 1e-6);
  EXPECT_NEAR(residual.value()(0), 0, 1e-6);
}
