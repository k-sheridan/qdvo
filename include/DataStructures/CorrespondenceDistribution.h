#ifndef CORRESPONDENCEDISTRIBUTION_H
#define CORRESPONDENCEDISTRIBUTION_H

#include <unordered_set>
#include <GlobalDefinitions.h>
#include <boost/geometry.hpp>
#include <GenericQuadTree.h>

#define OCCUPANCY_BIN_SIZE 5

namespace QDVO {

/*
 * The correspondence distribution is modeled as a gaussian mixture model. The means are of pixel resolution and have associated scores.
 * This class provides all the functionality necessary for efficiently adding to and evealuting the gaussian mixture model. It is meant to be preallocated once and held until the algorithm completes.
 */
class CorrespondenceDistribution
{
public:
    CorrespondenceDistribution();

    struct PotentialCorrespondence{
        SCALAR_TYPE score, x, y; // match score and location.
    };

    // serves as a method for finding nearest neighbors.
    GenericQuadTree<PotentialCorrespondence*> correspondenceMap;


    std::deque<PotentialCorrespondence> potentialCorrespondences; // stores the potential correspondences.

};

}

#endif // CORRESPONDENCEDISTRIBUTION_H
