#pragma once

#include "GlobalDefinitions.h"
#include "EpipolarDepthEstimator.h"
#include <Eigen/Core>

namespace  QDVO {
class Landmark
{
public:
    Landmark();

    Eigen::Matrix<SCALAR_TYPE, 3, 1> bearing;
    Eigen::Matrix<SCALAR_TYPE, 2, 1> px;
    SCALAR_TYPE dinv;

    ID_TYPE landmarkID, parentFrameID; // unique id for this landmark. unique id for the parent frame of this landmark.

    EpipolarDepthEstimator depthEstimator; // used to initialize the depth for this landmark

    enum LandmarkStatus{
        INACTIVE,
        ACTIVE,
        MARGINALIZED
    } status;

    Eigen::Matrix<SCALAR_TYPE, 3, 1> getEuclideanPoint();
};
}

