#include "CorrespondenceDistribution.h"

QDVO::CorrespondenceDistribution::CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<RadialSearchPattern> patternPtr)
{
    this->correspondenceMap = QDVO::SpatialMap<SpatialMapType>(std::max(width, height));
    this->radialSearchPattern = std::shared_ptr<RadialSearchPattern>(patternPtr);
}

Eigen::Matrix<SCALAR_TYPE, 2, 1> QDVO::CorrespondenceDistribution::computeResidual(const Eigen::Matrix<SCALAR_TYPE, 2, 1>& px_0)
{
    assert(!this->dormant);

    // Look for the closest potential correspondences approximately under a certain radius using the generic quadtree. While looking compute the gaussians
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence*> pcs = this->search(Eigen::Vector2i(px_0(0), px_0(1)), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, true);

    // compute the gaussian weights and residual finally



    Eigen::Matrix<SCALAR_TYPE, 2, 1> result;
    return result;
}

void QDVO::CorrespondenceDistribution::reset()
{
    this->patchComparer = nullptr; // ensure that we cannot use the wrong patch comparison
    this->dormant = true; // put this correspondence distribution to sleep.
    this->correspondenceMap.reset(); // wipe the actual distribution container clean.
}

void QDVO::CorrespondenceDistribution::initializeDistribution(const Eigen::Vector2i& centerPixel, const int floodRadius, std::shared_ptr<QDVO::PatchComparer> patchComparerPtr)
{
    this->dormant = false; // set the distribution to awake.

    this->patchComparer = std::shared_ptr<QDVO::PatchComparer>(patchComparerPtr); // set a new patch comparer for this distribution

    // perform the search
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence*> pcs = this->search(centerPixel, floodRadius, false);
}

std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence*> QDVO::CorrespondenceDistribution::search(const Eigen::Vector2i& centerPixel, const unsigned searchRadius, bool minimalSearch)
{
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence*> influentialPotentialCorrespondences;

    // start the radial search

    return influentialPotentialCorrespondences;
}
