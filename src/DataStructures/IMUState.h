#pragma once

#include <sophus/common.hpp>
#include <sophus/types.hpp>
#include <sophus/se3.hpp>
#include "GlobalDefinitions.h"
#include "Types.h"

namespace  QDVO {
class IMUState
{
public:
    IMUState();

    int dimensions = 9;

    double time; // the time of this state.

    QDVO::SO3 attitude;
    QDVO::Vector3 pos;
    QDVO::Vector3 vel;
    // Optionally, I may need angular velocity for the tightly coupled quadrotor integration.


    /**
     * @brief update the imustate (imu pose and velocity) with a generalized addition operator.
     * Order: [dp, dphi, dv];
     */
    void update(const Eigen::Matrix<SCALAR_TYPE, 9, 1>& dx);
    /**
     * @brief order: [dp, dphi]
     */
    void updatePose(const Eigen::Matrix<SCALAR_TYPE, 6, 1> &dx);
    /**
     * @brief order: [dv]
     */
    void updateVelocity(const Eigen::Matrix<SCALAR_TYPE, 3, 1> &dx);

    /**
     * creates a sophus se3 type representing the imu pose
     */
    QDVO::SE3 getSE3();
};
}

