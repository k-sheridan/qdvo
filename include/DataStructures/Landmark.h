#pragma once

#include <GlobalDefinitions.h>
#include <Eigen/Core>

class Landmark
{
public:
    Landmark();

    Eigen::Matrix<SCALAR_TYPE, 2, 1> bearing;
    Eigen::Matrix<SCALAR_TYPE, 2, 1> px;
    SCALAR_TYPE dinv;

    uint64_t landmarkID; // unique id for this landmark.

    enum LandmarkStatus{
        INACTIVE,
        ACTIVE,
        MARGINALIZED
    } status;
};

