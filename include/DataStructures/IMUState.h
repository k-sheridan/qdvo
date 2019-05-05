#pragma once

#include <sophus/common.hpp>
#include <sophus/types.hpp>
#include <sophus/se3.hpp>
#include <GlobalDefinitions.h>

class IMUState
{
public:
    IMUState();

    const int dimensions = IMUSTATE_DIMENSIONS;

    Sophus::SE3<SCALAR_TYPE> pose;
    Sophus::Vector3<SCALAR_TYPE> vel;
    // Optionally, I may need angular velocity for the tightly coupled quadrotor integration.


    /**
     * @brief update the imustate (imu pose and velocity) with a generalized addition operator.
     * Order: [dp, dphi, dv];
     */
    void update(const Eigen::Matrix<SCALAR_TYPE, IMUSTATE_DIMENSIONS, 1>& dx);
};

