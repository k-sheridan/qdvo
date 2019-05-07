#include "IMUState.h"

QDVO::IMUState::IMUState()
{

}

void QDVO::IMUState::update(const Eigen::Matrix<double, 9, 1> &dx)
{
    this->vel += dx.block(6, 0, 3, 1);
    this->pos += dx.block(0, 0, 3, 1);
    this->attitude *= Sophus::SO3<SCALAR_TYPE>(dx.block(3, 0, 3, 1));
}



