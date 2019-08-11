#include "CorrespondenceDistribution.h"
#include "PatchComparer.h"

QDVO::CorrespondenceDistribution::CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<RadialSearchPattern> patternPtr, QDVO::Frame *framePtr)
{
    this->correspondenceMap = QDVO::SpatialMap<PotentialCorrespondence>(std::max(width, height));
    this->radialSearchPattern = std::shared_ptr<RadialSearchPattern>(patternPtr);
    this->framePtr = framePtr;
}

Eigen::Matrix<SCALAR_TYPE, 2, 1> QDVO::CorrespondenceDistribution::computeResidual(const Eigen::Matrix<SCALAR_TYPE, 2, 1> &px_0)
{
    assert(!this->dormant);

    // Look for the closest potential correspondences approximately under a certain radius using the generic quadtree. While looking compute the gaussians
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> pcs = this->search(Eigen::Vector2i(px_0(0), px_0(1)), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, true);

    // compute the gaussian weights and residual finally

    Eigen::Matrix<SCALAR_TYPE, 2, 1> result;
    return result;
}

void QDVO::CorrespondenceDistribution::reset()
{
    this->patchComparer = nullptr;   // ensure that we cannot use the wrong patch comparison
    this->dormant = true;            // put this correspondence distribution to sleep.
    this->correspondenceMap.reset(); // wipe the actual distribution container clean.
}

void QDVO::CorrespondenceDistribution::initializeDistribution(const Eigen::Vector2i &centerPixel, const int floodRadius, std::shared_ptr<QDVO::PatchComparer> patchComparerPtr, QDVO::Patch warpedPatch)
{
    this->dormant = false; // set the distribution to awake.

    this->warpedPatch = warpedPatch; // replace the patch

    this->patchComparer = std::shared_ptr<QDVO::PatchComparer>(patchComparerPtr); // set a new patch comparer for this distribution

    // perform the search
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> pcs = this->search(centerPixel, floodRadius, false);
}

std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> QDVO::CorrespondenceDistribution::search(const Eigen::Vector2i &centerPixel, const unsigned searchRadius, bool minimalSearch)
{
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> influentialPotentialCorrespondences;
    bool firstInfluentialPotentialCorrespondenceFound = false;
    int radiusCounter = SEARCH_RADIUS_PADDING;
    assert(this->patchComparer != nullptr);

    // start the radial search
    for (unsigned r = 0; r <= searchRadius; ++r) // starting at radius 0 going to radius max.
    {
        // Is this the last radius to search?
        if (minimalSearch && firstInfluentialPotentialCorrespondenceFound)
        {
            if (radiusCounter-- <= 0)
            {
                break;
            }
        }

        for (auto &delta : this->radialSearchPattern->searchPattern.at(r))
        {
            Eigen::Vector2i testPoint = centerPixel + delta; // this is a point on a constant radius.

            // Make sure that this testPoint Is On The Image.
            if (!this->framePtr->cm->isPixelOnImage(testPoint))
            {
                continue;
            }

            auto &pc = this->correspondenceMap.get(testPoint);

            // Only run the patch comparison on uninitialized potential correspondences.
            if (!pc.initialized)
            {
                // try to compare the patch at the testPoint.
                auto score = this->patchComparer->compare(this->warpedPatch, *(this->framePtr), testPoint);

                // Add the score to the distribution
                if (score.has_value())
                {
                    pc.initialized = true;
                    pc.pixel = testPoint;
                    pc.score = score.value();

                    // If the potential correspondence is good enough, it is influential.
                    if (pc.score >= POTENTIAL_CORRESPONDENCE_THRESHOLD)
                    {
                        firstInfluentialPotentialCorrespondenceFound = true;
                        influentialPotentialCorrespondences.push_back(&pc);
                    }
                }
            }
        }
    }

    //std::cout << "found " << influentialPotentialCorrespondences.size() << std::endl;

    return influentialPotentialCorrespondences;
}
