#include "IMUState.h"

QDVO::IMUState::IMUState()
{

}

void QDVO::IMUState::update(const Eigen::Matrix<double, 9, 1> &dx)
{
    this->updatePose(dx.block(0, 0, 6, 1));
    this->updateVelocity(dx.block(6, 0, 3, 1));
}

void QDVO::IMUState::updatePose(const Eigen::Matrix<double, 6, 1> &dx)
{
    this->pos += dx.block(0, 0, 3, 1);
    this->attitude *= Sophus::SO3<SCALAR_TYPE>(dx.block(3, 0, 3, 1));
}

void QDVO::IMUState::updateVelocity(const Eigen::Matrix<double, 3, 1> &dx)
{
    this->vel += dx.block(6, 0, 3, 1);
}





