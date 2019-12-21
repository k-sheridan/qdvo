#pragma once

#include "GlobalDefinitions.h"
#include "EpipolarDepthEstimator.h"
#include "Types.h"
#include <Eigen/Core>

namespace  QDVO {
class Landmark
{
public:
    Landmark();

    /// A homogenous vector representing the bearing of the landmark in its parent frame.
    Eigen::Matrix<SCALAR_TYPE, 3, 1> bearing;
    /// The pixel space position of the feature in the parent image.
    Eigen::Matrix<SCALAR_TYPE, 2, 1> px;

    /// The inverse depth of the landmark in the parent frame.
    SCALAR_TYPE dinv;

    /// The key to the parent frame which this landmark is represented in.
    KeyframeMap::key_type parentFrameKey;

    /// A depth estimator used to initialize the depth of this landmark.
    EpipolarDepthEstimator depthEstimator;

    /// An enum used to mark the state of this landmark. 
    enum LandmarkStatus{
        INACTIVE,
        ACTIVE,
        MARGINALIZED
    } status;

    /// Compute the [x,y,z] position estimate of this landmark in the parent frame.
    Eigen::Matrix<SCALAR_TYPE, 3, 1> getEuclideanPoint();
};
}

