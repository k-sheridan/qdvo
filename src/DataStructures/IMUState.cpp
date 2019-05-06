#include "IMUState.h"

QDVO::IMUState::IMUState()
{

}

void QDVO::IMUState::update(const Eigen::Matrix<double, 9, 1> &dx)
{
    this->vel += dx.block(6, 0, 3, 1);
    this->pose *= Sophus::SE3<SCALAR_TYPE>(dx.block(0, 0, 6, 1));
}
