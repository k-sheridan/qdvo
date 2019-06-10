#include "CorrespondenceDistribution.h"

QDVO::CorrespondenceDistribution::CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<RadialSearchPattern> patternPtr)
{
    this->correspondenceMap = QDVO::SpatialMap<SpatialMapType>(std::max(width, height));
    this->radialSearchPattern = std::shared_ptr<RadialSearchPattern>(patternPtr);
}

Eigen::Matrix<SCALAR_TYPE, 2, 1> QDVO::CorrespondenceDistribution::computeResidual(const Eigen::Matrix<SCALAR_TYPE, 2, 1>& px_0)
{
    // Look for the closest potential correspondences approximately under a certain radius using the generic quadtree. While looking compute the gaussians

    Eigen::Matrix<SCALAR_TYPE, 2, 1> result;
    return result;
}
