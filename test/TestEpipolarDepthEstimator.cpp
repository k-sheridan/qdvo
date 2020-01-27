#include <gtest/gtest.h>

#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Image.h"
#include "GlobalDefinitions.h"
#include "PatchComparer.h"
#include "DataStructures/CorrespondenceDistribution.h"
#include "DataStructures/Image.h"
#include "GlobalDefinitions.h"
#include "PatchWarper.h"
#include "PatchComparer.h"
#include "CameraModel.hpp"
#include "EquidistantCameraModel.h"
#include "DataStructures/Frame.h"
#include "Types.h"

//class QDVOTestBench : public ::testing::Test {
//public:
//  void SetUp override {
//
//    cm = QDVO::EquidistantCameraModel(
//        300, 301, 255, 256, PI / 2.1, 512, 400,
//        Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257,
//                        -0.0020532361418706202, 0.00020293673591811182));
//
//    patchComparer =
//        std::make_shared<QDVO::PatchComparer>(QDVO::PatchComparer());
//  }
//
//  QDVO::EquidistantCameraModel cm;
//
//  std::shared_ptr<QDVO::PatchComparer> patchComparer;
//  std::shared_ptr<QDVO::PatchWarper> patchWarper;
//};
//
//TEST_F(QDVOTestBench, SearchForDepth){
//
//}
