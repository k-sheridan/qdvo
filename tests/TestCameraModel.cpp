#include "gtest/gtest.h"
#include "CameraModel.hpp"
#include "EquidistantCameraModel.h"

TEST(CameraModel, Basic)
{
    QDVO::CameraModel cm = QDVO::CameraModel(300, 301, 255, 256, PI/2.1, 512, 400);

    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 10);
    Eigen::Matrix2d projJac;
    Eigen::Vector2d px = cm.project(p);

    ASSERT_EQ(px.isApprox(Eigen::Vector2d(cm.cx, cm.cy)), true);

    px = cm.project(p, &projJac);

    ASSERT_NEAR(projJac(0, 0), cm.fx, SMALL_NUMBER);

    Eigen::Vector3d point = cm.unproject(px, &projJac);

    ASSERT_NEAR(projJac(0, 0), 1/cm.fx, SMALL_NUMBER);
    ASSERT_EQ(point.isApprox(Eigen::Vector3d(0, 0, 1)), true);

}

TEST(CameraModel, BreakingIt)
{
    QDVO::CameraModel cm = QDVO::CameraModel(300, 301, 255, 256, PI/3, 512, 400);

    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 0);
    Eigen::Matrix2d projJac;
    try {
        Eigen::Vector2d px = cm.project(p);
    } catch (std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

    p = Eigen::Vector3d(100, 0, 1);
    try {
        Eigen::Vector2d px = cm.project(p);
    } catch (std::runtime_error e) {
        std::cout << e.what() << std::endl;
    }

}


TEST(EQUIDISTANTCameraModel, Basic)
{
    QDVO::EquidistantCameraModel cm = QDVO::EquidistantCameraModel(300, 301, 255, 256, PI/2.1, 512, 400, Eigen::Vector4d(0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182));

    Eigen::Vector3d p = Eigen::Vector3d(0, 0, 10);
    Eigen::Matrix2d projJac;
    Eigen::Vector2d px = cm.project(p);



    //ASSERT_EQ(px.isApprox(Eigen::Vector2d(cm.cx, cm.cy)), true);

    px = cm.project(p, &projJac);

    std::cout << px << projJac;

    //ASSERT_NEAR(projJac(0, 0), cm.fx, SMALL_NUMBER);

    Eigen::Vector3d point = cm.unproject(px, &projJac);

    std::cout << point << projJac;

    //ASSERT_NEAR(projJac(0, 0), 1/cm.fx, SMALL_NUMBER);
    //ASSERT_EQ(point.isApprox(Eigen::Vector3d(0, 0, 1)), true);

}
