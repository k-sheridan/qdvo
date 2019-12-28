#include "Landmark.h"

QDVO::Landmark::Landmark()
{
    this->status = QDVO::Landmark::LandmarkStatus::INACTIVE;
}

Eigen::Matrix<SCALAR_TYPE, 3, 1> QDVO::Landmark::getEuclideanPoint() const
{
    assert(dinv > std::numeric_limits<SCALAR_TYPE>::min());

    return this->bearing / this->dinv;
}
