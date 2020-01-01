#include "CorrespondenceDistribution.h"
#include "Patch.h"
#include "PatchComparer.h"
#include "Landmark.h"
#include <algorithm>

QDVO::CorrespondenceDistribution::CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<const RadialSearchPattern> searchPattern)
{
    this->correspondenceMap = QDVO::SpatialMap<PotentialCorrespondence>(std::max(width, height));
    this->radialSearchPattern = std::move(searchPattern);
}

QDVO::Result<QDVO::Vector2> QDVO::CorrespondenceDistribution::computeResidual(CameraModel& cameraModel, Frame& frame, const QDVO::Vector2 &px_0)
{
    assert(!this->dormant);

    // Look for the closest potential correspondences approximately under a certain radius using the generic quadtree. While looking compute the gaussians
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> pcs = this->search(cameraModel, frame, Eigen::Vector2i(px_0(0), px_0(1)), MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, true);

    // compute the gaussian weights and residual finally
    errorArray.resize(pcs.size());
    std::transform(pcs.begin(), pcs.end(), errorArray.begin(), [px_0] (PotentialCorrespondence* pc) -> QDVO::Vector2 {
        assert(pc != nullptr); 
        return (px_0 - pc->pixel.cast<SCALAR_TYPE>());});

    expScoreArray.resize(pcs.size());
    std::transform(pcs.begin(), pcs.end(), errorArray.begin(), expScoreArray.begin(), [px_0] (PotentialCorrespondence* pc, QDVO::Vector2 error) -> SCALAR_TYPE {
        return exp(-0.5 * error.squaredNorm()) * pc->score;
    });

    SCALAR_TYPE gmm = std::accumulate(expScoreArray.begin(), expScoreArray.begin() + pcs.size(), 0);

    if (gmm < std::numeric_limits<SCALAR_TYPE>::min()){
        // This should never happen, but it could.
        throw std::runtime_error("gmm too small.");
    }

    weightedErrorArray.resize(pcs.size());

    std::transform(errorArray.begin(), errorArray.begin() + pcs.size(), expScoreArray.begin(), weightedErrorArray.begin(), 
    [gmm](QDVO::Vector2 error, SCALAR_TYPE expScore) -> QDVO::Vector2 {
        return (expScore / gmm) * error;
    });

    return std::accumulate(weightedErrorArray.begin(), weightedErrorArray.begin() + pcs.size(), QDVO::Vector2(0,0));
}

void QDVO::CorrespondenceDistribution::reset()
{
    this->patchComparer = nullptr;   // ensure that we cannot use the wrong patch comparison
    this->dormant = true;            // put this correspondence distribution to sleep.
    this->correspondenceMap.reset(); // wipe the actual distribution container clean.
}

void QDVO::CorrespondenceDistribution::initializeDistribution(CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey, const Eigen::Vector2i &centerPixel, const int floodRadius, std::shared_ptr<QDVO::PatchComparer> patchComparer, QDVO::Patch warpedPatch)
{
    this->dormant = false; // set the distribution to awake.

    this->warpedPatch = warpedPatch; // replace the patch

    this->patchComparer = std::move(patchComparer); // set a new patch comparer for this distribution

    this->landmarkKey = landmarkKey;

    // perform the search
    std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> pcs = this->search(cameraModel, frame, centerPixel, floodRadius, false);
}

std::vector<QDVO::CorrespondenceDistribution::PotentialCorrespondence *> QDVO::CorrespondenceDistribution::search(CameraModel& cameraModel, Frame& frame, const Eigen::Vector2i &centerPixel, const unsigned searchRadius, bool minimalSearch)
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
            if (!cameraModel.isPixelOnImage(testPoint))
            {
                continue;
            }

            QDVO::CorrespondenceDistribution::PotentialCorrespondence* pc = &(this->correspondenceMap.get(testPoint));

            // Only run the patch comparison on uninitialized potential correspondences.
            if (!pc->initialized)
            {
                // try to compare the patch at the testPoint.
                auto score = this->patchComparer->compare(this->warpedPatch, frame, testPoint);

                // Add the score to the distribution
                if(score.has_value())
                {
                    pc->initialized = true;
                    pc->pixel = testPoint;
                    pc->score = score.value();

                    // If the potential correspondence is good enough, it is influential.
                    if (pc->score >= POTENTIAL_CORRESPONDENCE_THRESHOLD)
                    {
                        firstInfluentialPotentialCorrespondenceFound = true;
                        influentialPotentialCorrespondences.push_back(pc);
                    }
                }
            }
        }
    }

    //std::cout << "found " << influentialPotentialCorrespondences.size() << std::endl;

    return influentialPotentialCorrespondences;
}
