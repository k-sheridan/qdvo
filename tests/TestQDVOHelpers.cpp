#include "gtest/gtest.h"
#include "VO.h"

TEST(CameraModelAdd, Basic)
{
    QDVO::VO vo;

    std::unique_ptr<QDVO::CameraModel> ptr(new QDVO::CameraModel(300, 301, 255, 256, PI/2.1));

    vo.setCameraModel(ptr, 1);

    ASSERT_EQ(vo.cameraModelMap.at(1).get()->cx, 255);
    ASSERT_EQ(ptr, nullptr);
}


