#pragma once

#include <sophus/common.hpp>
#include <sophus/types.hpp>
#include <sophus/se3.hpp>
#include <GlobalDefinitions.h>

namespace  QDVO {
class IMUState
{
public:
    IMUState();

    const int dimensions = 9;

    double time; // the time of this state.

    Sophus::SO3<SCALAR_TYPE> attitude;
    Sophus::Vector3<SCALAR_TYPE> pos;
    Sophus::Vector3<SCALAR_TYPE> vel;
    // Optionally, I may need angular velocity for the tightly coupled quadrotor integration.


    /**
     * @brief update the imustate (imu pose and velocity) with a generalized addition operator.
     * Order: [dp, dphi, dv];
     */
    void update(const Eigen::Matrix<SCALAR_TYPE, 9, 1>& dx);
    /**
     * @brief order: [dp, dphi]
     */
    void updatePose(const Eigen::Matrix<double, 6, 1> &dx);
    /**
     * @brief order: [dv]
     */
    void updateVelocity(const Eigen::Matrix<double, 3, 1> &dx);
};
}

