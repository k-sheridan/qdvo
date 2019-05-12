#include "gtest/gtest.h"
#include "CameraModel.hpp"

TEST(CameraModel, Basic)
{
    QDVO::CameraModel cm = QDVO::CameraModel(300, 301, 255, 256, PI/2.1);

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
    QDVO::CameraModel cm = QDVO::CameraModel(300, 301, 255, 256, PI/3);

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
