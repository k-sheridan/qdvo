#pragma once

#include <unordered_set>
#include <algorithm>
#include <memory>
#include <vector>
#include "GlobalDefinitions.h"
#include "RadialSearchPattern.h"
#include "SpatialMap.h"
#include "Patch.h"
#include "Types.h"


namespace QDVO {

class Frame;
class CameraModel;
class PatchComparer;

/**
 * The correspondence distribution is modeled as a gaussian mixture model. The means are of pixel resolution and have associated scores.
 * This class provides all the functionality necessary for efficiently adding to and evaluting the gaussian mixture model. It is meant to be preallocated once and held until the algorithm completes.
 */
class CorrespondenceDistribution
{
public:

    struct PotentialCorrespondence{
        SCALAR_TYPE score; // match score.
        Eigen::Vector2i pixel;
        bool initialized = false;

        void reset(){initialized = false;}
    };

    /// The warped template patch to be used for the creation of the correspondence distribution.
    Patch warpedPatch;

    /// Serves as a method for finding nearest neighbors.
    SpatialMap<PotentialCorrespondence> correspondenceMap;

    /// Is this correspondence distribution currently not being used.
    bool dormant = true; 

    /// The landmark this correspondence distribution represents an observation of.
    LandmarkMap::key_type landmarkKey; 


    CorrespondenceDistribution(unsigned width, unsigned height, std::shared_ptr<const RadialSearchPattern> patternPtr);

    /**
     * Will perform an initial radial search for potential correspondences to get an idea of the structure of the raw patch comparison function.
     * @return Number of valid potential correspondences during initialization.
     */
    int initializeDistribution(CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey, const Eigen::Vector2i& centerPixel, const int floodRadius, std::shared_ptr<QDVO::PatchComparer> patchComparerPtr, QDVO::Patch warpedPatch);

    /**
     * Efficiently evaluates the gradient of the negative log likelihood of the gaussian mixture model described by this class.
     */
    QDVO::Result<QDVO::Vector2> computeResidual(CameraModel& cameraModel, Frame& frame, const Eigen::Matrix<SCALAR_TYPE, 2, 1>& px_0);

    /**
     * clears all potential correspondences while retaining allocated memory, and sets the correspondence distribution into a dormant state.
     */
    void reset();

    /**
     * does a radial search while evaluating the patch comparison metric
     */
    std::vector<PotentialCorrespondence*> search(CameraModel& cameraModel, Frame& frame, const Eigen::Vector2i& centerPixel, const unsigned searchRadius, bool minimalSearch);

private:
    // pre-allocated quantities.
    std::vector<SCALAR_TYPE> expScoreArray;
    std::vector<QDVO::Vector2> errorArray, weightedErrorArray;

    std::shared_ptr<const QDVO::RadialSearchPattern> radialSearchPattern; // shared among all correspondence distributions. NOT TO BE MODIFIED!
    std::shared_ptr<QDVO::PatchComparer> patchComparer;


};



}
