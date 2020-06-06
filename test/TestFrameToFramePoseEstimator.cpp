#include <gtest/gtest.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "Estimation/FrameToFramePoseEstimator.h"
#include "Logging.h"
#include "Optimizer/Containers.h"
#include "Optimizer/ErrorTermValidator.h"
#include "TestFixtures.h"

class FrameToFramePoseEstimatorTest : public QDVOSyntheticImageTest {};

TEST_F(FrameToFramePoseEstimatorTest, validateDirectErrorTerm) {
  const std::string datasetPath =
      boost::filesystem::current_path().string() +
      "/../test/qdvo-test-datasets/dataset-room1_512_16_chopped/";

  // run qdvo
  cv::Mat img =
      cv::imread(datasetPath + "mav0/cam0/data/1520530308199447626.png",
                 cv::IMREAD_GRAYSCALE);

  auto frameKey = insertFrame();
  auto otherFrameKey = insertFrame();
  Eigen::Vector3d bearing = {0.3, 0.5, 1};
  double dinv = 0.5;
  auto landmarkKey = insertLandmark(frameKey, bearing, dinv);
  insertBlankImageIntoKeyframe(frameKey);
  insertBlankImageIntoKeyframe(otherFrameKey);

  (*graph.getKeyframeMap().at(frameKey))->imagePyr.generate(img);
  (*graph.getKeyframeMap().at(otherFrameKey))->imagePyr.generate(img);

  Eigen::Vector3d pt = bearing / dinv;

  // renderEdgeLandmark(frameKey, pt, {0, 1, 0}, 2, 0.0001, 100);
  // renderEdgeLandmark(otherFrameKey, pt, {0, 1, 0}, 2, 0.0001, 100);

  ArgMin::SO3 host;

  LOG_INFO("Creating SO3");

  host.value = Sophus::SO3d::exp({0, 0, 0});

  ArgMin::VariableContainer<ArgMin::SO3> vc;
  auto hostKey = vc.insert(host);

  LOG_INFO("Inserted variable");

  auto& frame = *(*graph.getKeyframeMap().at(frameKey));
  auto& otherFrame = *(*graph.getKeyframeMap().at(otherFrameKey));

  LOG_INFO("Got frames.");

  int imageLevel = frame.imagePyr.levels() - 1;
  imageLevel = 3;
  double ratio = std::pow(2, imageLevel);

  LOG_INFO("ratio: {}", ratio);

  const QDVO::Image& image1 = frame.imagePyr.getImage(imageLevel);
  const QDVO::Image& image2 = otherFrame.imagePyr.getImage(imageLevel);

  LOG_INFO("Got images");

  DirectSE2ErrorTerm::SharedData data = {graph, frameKey, otherFrameKey,
                                         ratio, image1,   image2};

  auto px = graph.getLandmarkMap().at(landmarkKey)->px;

  LOG_INFO("Got pixel: {}", px);

  DirectSE2ErrorTerm et(data, (px / ratio).cast<int>(), hostKey);

  et.updateVariablePointers(vc);

  et.evaluate(vc, true);

  LOG_INFO("J: {}", std::get<0>(et.variableJacobians));

  EXPECT_NEAR(et.residual(0, 0), 0, 1e-3);

  EXPECT_TRUE(et.linearizationValid);

  ArgMin::ErrorTermValidator<DirectSE2ErrorTerm> validator(et);

  EXPECT_TRUE(validator.validate(vc));
}

TEST_F(FrameToFramePoseEstimatorTest, estimateFrameToFrameTransform) {
  const std::string datasetPath =
      boost::filesystem::current_path().string() +
      "/../test/qdvo-test-datasets/dataset-room1_512_16_chopped/";

  // run qdvo
  cv::Mat img =
      cv::imread(datasetPath + "mav0/cam0/data/1520530330450787328.png",
                 cv::IMREAD_GRAYSCALE);

  cv::Mat img2 =
      cv::imread(datasetPath + "mav0/cam0/data/1520530330500788328.png",
                 cv::IMREAD_GRAYSCALE);

  auto frameKey = insertFrame();
  auto otherFrameKey = insertFrame();
  Eigen::Vector3d bearing = {0.3, 0.1, 1};
  double dinv = 0.5;
  auto landmarkKey = insertLandmark(frameKey, bearing, dinv);
  insertBlankImageIntoKeyframe(frameKey);
  insertBlankImageIntoKeyframe(otherFrameKey);

  (*graph.getKeyframeMap().at(frameKey))->imagePyr.generate(img);
  (*graph.getKeyframeMap().at(otherFrameKey))->imagePyr.generate(img2);

  QDVO::SE3 T_1_2;

  QDVO::FrameToFramePoseEstimator pe;

  EXPECT_TRUE(pe.estimateRelativePose(graph, frameKey, otherFrameKey, T_1_2));

  LOG_INFO("R: {}", T_1_2.so3().matrix());
  LOG_INFO("q: {}, {}, {}, {}", T_1_2.so3().unit_quaternion().w(),
           T_1_2.so3().unit_quaternion().x(), T_1_2.so3().unit_quaternion().y(),
           T_1_2.so3().unit_quaternion().z());

  // 1520530330456345351,0.3579356393,-0.3583713866,1.3926355658,0.9476594234,0.0289454746,0.1375011332,-0.2867005668
  // 1520530330506345351,0.3471259955,-0.3523329672,1.4022983242,0.9437967157,0.0285799143,0.1961106120,-0.2645214089
  Sophus::SO3d R1 = Sophus::SO3d(Eigen::Quaterniond(
      0.9476594234, 0.0289454746, 0.1375011332, -0.2867005668));
  Sophus::SO3d R2 = Sophus::SO3d(Eigen::Quaterniond(
      0.9437967157, 0.0285799143, 0.1961106120, -0.2645214089));

  LOG_INFO("est rotation: {} gt rotation: {}", T_1_2.so3().log().norm(),
           (R1.inverse() * R2).log().norm());

  EXPECT_NEAR(T_1_2.so3().log().norm(), (R1.inverse() * R2).log().norm(), 2e-2);
}
