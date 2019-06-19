#ifndef CORRESPONDENCEDISTRIBUTION_H
#define CORRESPONDENCEDISTRIBUTION_H

#include <unordered_set>
#include "GlobalDefinitions.h"
#include <boost/geometry.hpp>
#include <algorithm>
#include "RadialSearchPattern.h"
#include "SpatialMap.h"
#include "Patch.h"


namespace QDVO {

class Frame;

/*
 * The correspondence distribution is modeled as a gaussian mixture model. The means are of pixel resolution and have associated scores.
 * This class provides all the functionality necessary for efficiently adding to and evaluting the gaussian mixture model. It is meant to be preallocated once and held until the algorithm completes.
 */
class CorrespondenceDistribution
{
public:

    struct PotentialCorrespondence{
        SCALAR_TYPE score; // match score.
        Eigen::Vector2i pixel;
    };

    struct SpatialMapType {
        PotentialCorrespondence* pc = nullptr;
    };

    Patch warpedPatch; // the warped template patch to be used for the creation of the correspondence distribution.

    // serves as a method for finding nearest neighbors.
    SpatialMap<SpatialMapType> correspondenceMap;
    std::deque<PotentialCorrespondence> potentialCorrespondences; // stores the potential correspondences.





    CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<RadialSearchPattern> patternPtr = nullptr);

    /*
     * Efficiently evaluates the gradient of the negative log likelihood of the gaussian mixture model described by this class.
     */
    Eigen::Matrix<SCALAR_TYPE, 2, 1> computeResidual(const Eigen::Matrix<SCALAR_TYPE, 2, 1>& px_0);




private:

    // pre-allocated quantities.
    std::vector<SCALAR_TYPE> errorArray, errorSqArray, scoreArray, expScoreArray, weightArray;

    std::shared_ptr<RadialSearchPattern> radialSearchPattern; // shared among all correspondence distributions. NOT TO BE MODIFIED!
};



}

#endif // CORRESPONDENCEDISTRIBUTION_H
