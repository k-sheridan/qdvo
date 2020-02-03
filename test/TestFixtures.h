#include <gtest/gtest.h>

#include "CameraModel.hpp"
#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Frame.h"
#include "DataStructures/Graph.h"
#include "DataStructures/Image.h"
#include "EquidistantCameraModel.h"
#include "GlobalDefinitions.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "Types.h"

class QDVOBasicTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  QDVO::CameraModelMap::key_type addCameraToGraph(
      std::unique_ptr<QDVO::CameraModel> cameraModel) {
    QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                   Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
    return graph.insertCamera(std::move(cameraModel), unit);
  }

  std::unique_ptr<QDVO::EquidistantCameraModel> cm =
      std::make_unique<QDVO::EquidistantCameraModel>(
          300, 301, 255, 256, PI / 2.1, 512, 400,
          Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257,
                          -0.0020532361418706202, 0.00020293673591811182));

  std::shared_ptr<QDVO::PatchComparer> patchComparer =
      std::make_shared<QDVO::PatchComparer>(QDVO::PatchComparer());

  std::shared_ptr<QDVO::PatchWarper> patchWarper;

  QDVO::Graph graph;
};
