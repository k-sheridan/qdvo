#include <gtest/gtest.h>

#include "TestFixtures.h"

class EpipolarDepthEstimationTest : public QDVOSimpleGraphTest {};

/**
 * This test will simulate a epipolar depth search by creating two
 * images with a corner feature.
 */
TEST_F(EpipolarDepthEstimationTest, DISABLED_EstimateDepth) {
  // Insert a camera model into the graph.
  //  auto cameraModelKey =
  //  addCameraToGraph(std::make_unique<QDVO::EquidistantCameraModel>(*cm));

  // Create an extrinsic for the keyframes.
  QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                 Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
  auto extrinsicKey = graph.getExtrinsicMap().insert(unit);

  // Create source keyframe with image and landmark.
  auto sourceKeyframeKey =
      graph.getKeyframeMap().insert(std::move(std::make_unique<QDVO::Frame>()));
  auto& sourceKeyframe = *(*graph.getKeyframeMap().at(sourceKeyframeKey));

  // Set up the source keyframe pose.
  sourceKeyframe.imustate.pos = QDVO::Vector3(0, 0, 0);
  EXPECT_EQ(sourceKeyframe.imustate.attitude.unit_quaternion().w(), 1.0);

  // Insert a landmark into the sourceKeyframe.
  auto landmarkKey = graph.getLandmarkMap().insert(QDVO::Landmark());
  auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
  landmark.parentFrameKey = sourceKeyframeKey;
  sourceKeyframe.landmarkKeys.push_back(landmarkKey);

  // Set the landmark bearing and dinv.
  landmark.bearing = QDVO::Vector3(0, 0, 1);
  double depthGT = 2;
  landmark.dinv = 1 / depthGT;

  // Set the source keyframe's extrinsic key.
  sourceKeyframe.extrinsicKey = extrinsicKey;

  // Set the source frame's camera model.
  sourceKeyframe.cameraModelKey = cameraModelKey;

  // Compute the source pixel position.
  auto sourcePixelResult =
      graph.projectLandmarkToPixel(sourceKeyframe, sourceKeyframe, landmark);
  ASSERT_TRUE(sourcePixelResult.has_value());

  auto landmarkInSource = graph.projectLandmarkToCameraFrame(
      sourceKeyframe, sourceKeyframe, landmark);
  EXPECT_EQ(landmarkInSource(0), 0);
  EXPECT_EQ(landmarkInSource(1), 0);
  EXPECT_EQ(landmarkInSource(2), 2);

  // Set the source pixel position.
  landmark.px = sourcePixelResult.value();

  // Create the source image.
  QDVO::Image image;
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();
  // Set a single pixel in the center to high.
  image.getImageData()(static_cast<Eigen::Index>(std::round(landmark.px(1))),
                       static_cast<Eigen::Index>(std::round(landmark.px(0)))) =
      1.0;

  // Update the source keyframe's image.
  cv::Mat cvMat;
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  sourceKeyframe.updateImage(cvMat);

  // Create the target keyframe.
  auto targetKeyframeKey =
      graph.getKeyframeMap().insert(std::move(std::make_unique<QDVO::Frame>()));
  auto& targetKeyframe = *(*graph.getKeyframeMap().at(targetKeyframeKey));

  // Set the target keyframe's extrinsic key.
  targetKeyframe.extrinsicKey = extrinsicKey;

  // Set the target frame's camera model.
  targetKeyframe.cameraModelKey = cameraModelKey;

  // Set up the target keyframe pose.
  targetKeyframe.imustate.pos = QDVO::Vector3(1, 0, 0);

  // Create the target image.
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();

  // Compute the pixel position of the landmark.
  auto targetPixelResult =
      graph.projectLandmarkToPixel(targetKeyframe, sourceKeyframe, landmark);
  ASSERT_TRUE(targetPixelResult.has_value());
  auto targetPixel = targetPixelResult.value();

  auto landmarkInTarget = graph.projectLandmarkToCameraFrame(
      targetKeyframe, sourceKeyframe, landmark);
  EXPECT_EQ(landmarkInTarget(0), -1);
  EXPECT_EQ(landmarkInTarget(1), 0);
  EXPECT_EQ(landmarkInTarget(2), 2);

  std::cout << std::endl << targetPixel << landmark.px << std::endl;

  // Set a single pixel in the center to high.
  image.getImageData()(static_cast<Eigen::Index>(std::round(targetPixel(1))),
                       static_cast<Eigen::Index>(std::round(targetPixel(0)))) =
      1.0;

  // Update the source keyframe's image.
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  targetKeyframe.updateImage(cvMat);

  EXPECT_NE(targetKeyframe.imagePyr.getImage().getImageData().sum(), 0);

  // Before running the depth estimator, set the landmark depth to the incorrect
  // value.
  landmark.dinv = 1e-3;

  // Run the epipolar depth estimator with the targetKeyframe.
  landmark.depthEstimator.update(graph, sourceKeyframe, targetKeyframe,
                                 landmark, *patchComparer, *patchWarper);

  // The depth estimator should have updated the depth to near the solution.
  EXPECT_LT(std::abs(1 / landmark.dinv - depthGT), 0.1);
  // The landmark should now be initialized.
  EXPECT_TRUE(landmark.depthEstimator.initialized);
  // The landmark should not be marked MARGINALIZED.
  EXPECT_NE(landmark.status, QDVO::Landmark::LandmarkStatus::MARGINALIZED);
}

/**
 * This test verifies that the epipolar depth estimator marks the
 * landmark as an outlier feature if it has no matches.
 */
TEST_F(EpipolarDepthEstimationTest, DISABLED_EstimateDepthWithNoMatches) {
  // Insert a camera model into the graph.
  //  auto cameraModelKey =
  //  addCameraToGraph(std::make_unique<QDVO::EquidistantCameraModel>(*cm));

  // Create an extrinsic for the keyframes.
  QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                 Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
  auto extrinsicKey = graph.getExtrinsicMap().insert(unit);

  // Create source keyframe with image and landmark.
  auto sourceKeyframeKey =
      graph.getKeyframeMap().insert(std::move(std::make_unique<QDVO::Frame>()));
  auto& sourceKeyframe = *(*graph.getKeyframeMap().at(sourceKeyframeKey));

  // Set up the source keyframe pose.
  sourceKeyframe.imustate.pos = QDVO::Vector3(0, 0, 0);
  EXPECT_EQ(sourceKeyframe.imustate.attitude.unit_quaternion().w(), 1.0);

  // Insert a landmark into the sourceKeyframe.
  auto landmarkKey = graph.getLandmarkMap().insert(QDVO::Landmark());
  auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
  landmark.parentFrameKey = sourceKeyframeKey;
  sourceKeyframe.landmarkKeys.push_back(landmarkKey);

  // Set the landmark bearing and dinv.
  landmark.bearing = QDVO::Vector3(0, 0, 1);
  double depthGT = 2;
  landmark.dinv = 1 / depthGT;

  // Set the source keyframe's extrinsic key.
  sourceKeyframe.extrinsicKey = extrinsicKey;

  // Set the source frame's camera model.
  sourceKeyframe.cameraModelKey = cameraModelKey;

  // Compute the source pixel position.
  auto sourcePixelResult =
      graph.projectLandmarkToPixel(sourceKeyframe, sourceKeyframe, landmark);
  ASSERT_TRUE(sourcePixelResult.has_value());

  auto landmarkInSource = graph.projectLandmarkToCameraFrame(
      sourceKeyframe, sourceKeyframe, landmark);
  EXPECT_EQ(landmarkInSource(0), 0);
  EXPECT_EQ(landmarkInSource(1), 0);
  EXPECT_EQ(landmarkInSource(2), 2);

  // Set the source pixel position.
  landmark.px = sourcePixelResult.value();

  // Create the source image.
  QDVO::Image image;
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();
  // Set a single pixel in the center to high.
  image.getImageData()(static_cast<Eigen::Index>(std::round(landmark.px(1))),
                       static_cast<Eigen::Index>(std::round(landmark.px(0)))) =
      1.0;

  // Update the source keyframe's image.
  cv::Mat cvMat;
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  sourceKeyframe.updateImage(cvMat);

  // Create the target keyframe.
  auto targetKeyframeKey =
      graph.getKeyframeMap().insert(std::move(std::make_unique<QDVO::Frame>()));
  auto& targetKeyframe = *(*graph.getKeyframeMap().at(targetKeyframeKey));

  // Set the target keyframe's extrinsic key.
  targetKeyframe.extrinsicKey = extrinsicKey;

  // Set the target frame's camera model.
  targetKeyframe.cameraModelKey = cameraModelKey;

  // Set up the target keyframe pose.
  targetKeyframe.imustate.pos = QDVO::Vector3(1, 0, 0);

  // Create the target image.
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();

  // Compute the pixel position of the landmark.
  auto targetPixelResult =
      graph.projectLandmarkToPixel(targetKeyframe, sourceKeyframe, landmark);
  ASSERT_TRUE(targetPixelResult.has_value());
  auto targetPixel = targetPixelResult.value();

  auto landmarkInTarget = graph.projectLandmarkToCameraFrame(
      targetKeyframe, sourceKeyframe, landmark);
  EXPECT_EQ(landmarkInTarget(0), -1);
  EXPECT_EQ(landmarkInTarget(1), 0);
  EXPECT_EQ(landmarkInTarget(2), 2);

  // Update the source keyframe's image.
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  targetKeyframe.updateImage(cvMat);

  // Before running the depth estimator, set the landmark depth to the incorrect
  // value.
  landmark.dinv = 1e-3;

  // Run the epipolar depth estimator with the targetKeyframe.
  landmark.depthEstimator.update(graph, sourceKeyframe, targetKeyframe,
                                 landmark, *patchComparer, *patchWarper);

  // The landmark should now be initialized.
  EXPECT_FALSE(landmark.depthEstimator.initialized);
  // The landmark should not be marked MARGINALIZED.
  EXPECT_EQ(landmark.status, QDVO::Landmark::LandmarkStatus::MARGINALIZED);
}

/**
 * This test will verify that the estimator is not initialized
 * when too many matches are not unique by simulating an edge feature.
 */
TEST_F(EpipolarDepthEstimationTest, EstimateDepthAlongEdge) {
  // Insert a camera model into the graph.
  //  auto cameraModelKey =
  //  addCameraToGraph(std::make_unique<QDVO::EquidistantCameraModel>(*cm));

  // Create an extrinsic for the keyframes.
  QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                 Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
  auto extrinsicKey = graph.getExtrinsicMap().insert(unit);

  // Create source keyframe with image and landmark.
  auto sourceKeyframeKey =
      graph.getKeyframeMap().insert(std::move(std::make_unique<QDVO::Frame>()));
  auto& sourceKeyframe = *(*graph.getKeyframeMap().at(sourceKeyframeKey));

  // Set up the source keyframe pose.
  sourceKeyframe.imustate.pos = QDVO::Vector3(0, 0, 0);
  EXPECT_EQ(sourceKeyframe.imustate.attitude.unit_quaternion().w(), 1.0);

  // Insert a landmark into the sourceKeyframe.
  auto landmarkKey = graph.getLandmarkMap().insert(QDVO::Landmark());
  auto& landmark = *graph.getLandmarkMap().at(landmarkKey);
  landmark.parentFrameKey = sourceKeyframeKey;
  sourceKeyframe.landmarkKeys.push_back(landmarkKey);

  // Set the landmark bearing and dinv.
  landmark.bearing = QDVO::Vector3(0, 0, 1);
  double depthGT = 2;
  landmark.dinv = 1 / depthGT;

  // Set the source keyframe's extrinsic key.
  sourceKeyframe.extrinsicKey = extrinsicKey;

  // Set the source frame's camera model.
  sourceKeyframe.cameraModelKey = cameraModelKey;

  // Compute the source pixel position.
  auto sourcePixelResult =
      graph.projectLandmarkToPixel(sourceKeyframe, sourceKeyframe, landmark);
  ASSERT_TRUE(sourcePixelResult.has_value());

  auto landmarkInSource = graph.projectLandmarkToCameraFrame(
      sourceKeyframe, sourceKeyframe, landmark);
  EXPECT_EQ(landmarkInSource(0), 0);
  EXPECT_EQ(landmarkInSource(1), 0);
  EXPECT_EQ(landmarkInSource(2), 2);

  // Set the source pixel position.
  landmark.px = sourcePixelResult.value();

  // Create the source image.
  QDVO::Image image;
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();
  // Make a line.
  image.getImageData()
      .block(std::round(landmark.px(1)), 0, 1, 512)
      .setConstant(1);

  // Update the source keyframe's image.
  cv::Mat cvMat;
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  sourceKeyframe.updateImage(cvMat);

  // Create the target keyframe.
  auto targetKeyframeKey =
      graph.getKeyframeMap().insert(std::move(std::make_unique<QDVO::Frame>()));
  auto& targetKeyframe = *(*graph.getKeyframeMap().at(targetKeyframeKey));

  // Set the target keyframe's extrinsic key.
  targetKeyframe.extrinsicKey = extrinsicKey;

  // Set the target frame's camera model.
  targetKeyframe.cameraModelKey = cameraModelKey;

  // Set up the target keyframe pose.
  targetKeyframe.imustate.pos = QDVO::Vector3(1, 0, 0);

  // Create the target image.
  image.getImageData() = Eigen::MatrixXf(512, 512);
  image.getImageData().setZero();

  // Compute the pixel position of the landmark.
  auto targetPixelResult =
      graph.projectLandmarkToPixel(targetKeyframe, sourceKeyframe, landmark);
  ASSERT_TRUE(targetPixelResult.has_value());
  auto targetPixel = targetPixelResult.value();

  auto landmarkInTarget = graph.projectLandmarkToCameraFrame(
      targetKeyframe, sourceKeyframe, landmark);
  EXPECT_EQ(landmarkInTarget(0), -1);
  EXPECT_EQ(landmarkInTarget(1), 0);
  EXPECT_EQ(landmarkInTarget(2), 2);

  std::cout << std::endl << targetPixel << landmark.px << std::endl;

  // Make a line.
  image.getImageData()
      .block(std::round(targetPixel(1)), 0, 1, 512)
      .setConstant(1);

  // Update the source keyframe's image.
  image.toOpenCVImage().convertTo(cvMat, CV_16U);
  targetKeyframe.updateImage(cvMat);

  // Before running the depth estimator, set the landmark depth to the incorrect
  // value.
  landmark.dinv = 1e-3;

  // Run the epipolar depth estimator with the targetKeyframe.
  landmark.depthEstimator.update(graph, sourceKeyframe, targetKeyframe,
                                 landmark, *patchComparer, *patchWarper);

  // The landmark should now be initialized.
  EXPECT_FALSE(landmark.depthEstimator.initialized);
  // The landmark should not be marked MARGINALIZED.
  // Since this is the first attempt.
  EXPECT_NE(landmark.status, QDVO::Landmark::LandmarkStatus::MARGINALIZED);
}
