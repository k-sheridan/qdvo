#pragma once

#include "GlobalDefinitions.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "DataStructures/Frame.h"
#include "DataStructures/Patch.h"
#include "Types.h"
#include <Eigen/Core>

namespace QDVO {

/**
 * base class used to compare two image patches.
 * For speed, this is implemented in a way which will compare a patch with a pixel, image combinination.
 * This base class is implemented as a ZNCC.
 */
class PatchComparer
{
public:

    PatchComparer();

    /**
     * uses the zero mean normalized cross correlation to determine how well a target frame pixel matches the patch.
     * result is bounded on the interval [0, 1] where 0 is a highly unlikely match, and 1 is a highly likely match.
     *
     * no out of bounds checks are performed at this level.
     */
    virtual QDVO::Result<SCALAR_TYPE> compare(QDVO::Patch& templatePatch, Frame& targetFrame, Eigen::Vector2i& pixel);

    /**
     * Compares two patches and returns a score on [0,1]
     */
    virtual QDVO::Result<SCALAR_TYPE> compare(QDVO::Patch& templatePatch, QDVO::Patch& targetPatch);

};
}
