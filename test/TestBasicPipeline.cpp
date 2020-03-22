#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "BasicPipeline.h"
#include "TestFixtures.h"

/**
 * Test fixture used to test BasicPipeline with the TUM VI datasets.
 */
class BasicPipelineTest : public ::testing::Test {
  void SetUp() {
    pipeline.initialize();

    std::unique_ptr<QDVO::CameraModel> cm(
        std::make_unique<QDVO::EquidistantCameraModel>(
            190.97847715128717, 190.9733070521226, 254.93170605935475,
            256.8974428996504, 1.44 * 2, 512, 512,
            Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257,
                            -0.0020532361418706202, 0.00020293673591811182)));

    cameraModelKey = pipeline.addCamera(std::move(cm));

    extrinsicKey = pipeline.graph.getExtrinsicMap().insert(
        pipeline.graph.getCameraModelMap().at(cameraModelKey)->second);
  }

 public:
  QDVO::BasicPipeline pipeline;

  QDVO::CameraModelMap::key_type cameraModelKey;
  QDVO::ExtrinsicMap::key_type extrinsicKey;
};

TEST_F(BasicPipelineTest, RunDataset) {
  const std::string datasetPath =
      std::filesystem::current_path().string() +
      "/../test/qdvo-test-datasets/dataset-room1_512_16_chopped/";
  std::string cam0CsvPath =
      datasetPath +
      "mav0/cam0/data.csv";
  std::ifstream cam0CSV;
  cam0CSV.open(cam0CsvPath, std::ifstream::in);
  if (!cam0CSV.is_open()) {
    LOG_ERROR("Failed to open dataset csv. {}", cam0CsvPath);
    return;
  }

  const int maxFrames = 1000;
  int frameCount = 1;
  std::string csvLine;
  std::getline(cam0CSV, csvLine);
  std::getline(cam0CSV, csvLine);

  while (csvLine.size()) {
    // find the time and file of the next image.
    std::stringstream ss(csvLine);
    std::string timeStr, fileStr;
    std::getline(ss, timeStr, ',');
    std::getline(ss, fileStr, ',');

    uint64_t t = std::stoull(timeStr);  // nanoseconds

    // run qdvo
    cv::Mat img = cv::imread(datasetPath + "mav0/cam0/data/" + fileStr,
                             cv::IMREAD_GRAYSCALE);

    pipeline.addFrame(img, t / 1e-9, cameraModelKey, extrinsicKey);

    // increment csv
    std::getline(cam0CSV, csvLine);

    ++frameCount;

    if (frameCount >= maxFrames) {
      break;
    }
  }
}
