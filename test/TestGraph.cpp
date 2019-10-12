#include "gtest/gtest.h"
#include "Graph.h"

TEST(CameraModelAdd, Basic)
{
    QDVO::Graph g;

    std::unique_ptr<QDVO::CameraModel> ptr(new QDVO::CameraModel(300, 301, 255, 256, PI/2.1, 512, 400));

    g.setCameraModel(ptr, 1);

    //ASSERT_EQ(g.cameraModelMap.at(1).get()->cx, 255);
    //ASSERT_EQ(ptr, nullptr);
}


