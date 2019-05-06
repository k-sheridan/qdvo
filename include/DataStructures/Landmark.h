#pragma once

#include <GlobalDefinitions.h>
#include <Eigen/Core>

namespace  QDVO {
class Landmark
{
public:
    Landmark();

    Eigen::Matrix<SCALAR_TYPE, 2, 1> bearing;
    Eigen::Matrix<SCALAR_TYPE, 2, 1> px;
    SCALAR_TYPE dinv;

    ID_TYPE landmarkID, parentFrameID; // unique id for this landmark. unique id for the parent frame of this landmark.

    enum LandmarkStatus{
        INACTIVE,
        ACTIVE,
        MARGINALIZED
    } status;
};
}

