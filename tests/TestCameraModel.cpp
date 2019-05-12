#include "gtest/gtest.h"
#include "CameraModel.hpp"

TEST(CameraModel, Basic)
{
    QDVO::CameraModel cm = QDVO::CameraModel();
    cm.cx = 256;
    cm.cy = 256;
    cm.fx = 300;
    cm.fy = 300;

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
