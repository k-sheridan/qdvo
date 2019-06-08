#ifndef CORRESPONDENCEDISTRIBUTION_H
#define CORRESPONDENCEDISTRIBUTION_H

#include <unordered_set>
#include <GlobalDefinitions.h>
#include <boost/geometry.hpp>
#include <GenericQuadTree.h>
#include <algorithm>
#include <RadialSearchPattern.h>
#include <SpatialMap.h>


#define OCCUPANCY_BIN_SIZE 5

namespace QDVO {

/*
 * The correspondence distribution is modeled as a gaussian mixture model. The means are of pixel resolution and have associated scores.
 * This class provides all the functionality necessary for efficiently adding to and evaluting the gaussian mixture model. It is meant to be preallocated once and held until the algorithm completes.
 */
class CorrespondenceDistribution
{
public:
    CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<RadialSearchPattern> patternPtr = nullptr);

    struct PotentialCorrespondence{
        SCALAR_TYPE score; // match score.
        Eigen::Vector2i pixel;
    };

    struct SpatialMapType{
        PotentialCorrespondence* pc = nullptr;
        bool searched = false;
    };

    /*
     * Inserts a potential correspondence in to the distribution. This will be used for the computation of the residual as described in my paper.
     */
    void addPotentialCorrespondence(const PotentialCorrespondence& pc);

    /*
     * Efficiently evaluates the gradient of the negative log likelihood of the gaussian mixture model described by this class.
     */
    Eigen::Matrix<SCALAR_TYPE, 2, 1> computeResidual(const Eigen::Matrix<SCALAR_TYPE, 2, 1>& px_0);

    // VARIABLES

    // serves as a method for finding nearest neighbors.
    SpatialMap<PotentialCorrespondence*> correspondenceMap;


    std::deque<PotentialCorrespondence> potentialCorrespondences; // stores the potential correspondences.

    // pre-allocated quantities.
    std::vector<SCALAR_TYPE> errorArray, errorSqArray, scoreArray, expScoreArray, weightArray;

    std::shared_ptr<RadialSearchPattern> radialSearchPattern; // shared among all correspondence distributions. NOT TO BE MODIFIED!
};



}

#endif // CORRESPONDENCEDISTRIBUTION_H
