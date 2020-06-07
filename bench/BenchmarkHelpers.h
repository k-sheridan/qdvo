#pragma once

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "CameraModel.hpp"
#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Frame.h"
#include "DataStructures/Graph.h"
#include "DataStructures/Image.h"
#include "DataStructures/RadialSearchPattern.h"
#include "EquidistantCameraModel.h"
#include "GlobalDefinitions.h"
#include "Logging.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "Types.h"

/// Load a test image.
cv::Mat getTestImage() {
  const std::string datasetPath =
      boost::filesystem::current_path().string() +
      "/../test/qdvo-test-datasets/dataset-room1_512_16_chopped/";

  // run qdvo
  cv::Mat img =
      cv::imread(datasetPath + "mav0/cam0/data/1520530308199447626.png",
                 cv::IMREAD_GRAYSCALE);
  return img;
}

std::unique_ptr<QDVO::CameraModel> getTestCameraModel() {
  std::unique_ptr<QDVO::EquidistantCameraModel> cm =
      std::make_unique<QDVO::EquidistantCameraModel>(
          300, 301, 255, 256, PI / 2.1, 512, 400,
          Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257,
                          -0.0020532361418706202, 0.00020293673591811182));
  return cm;
}
