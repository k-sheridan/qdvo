#ifndef PATCHWARPER_H
#define PATCHWARPER_H

#include "GlobalDefinitions.h"
#include <opencv2/core.hpp>
#include "Frame.h"
#include "Patch.h"
#include "Graph.h"

namespace QDVO {
/*
 * This class is used to warp a feature represented as a patch into a new frame.
 * the patch normal is assumed to be oriented towards the center of the baseline between the source and target frame.
 */
class PatchWarper
{
public:
    PatchWarper();

    /*
     * computes a warped patch in the target frame assuming the feature lies on a flat surface.
     */
    void warpPatchToTargetFrame(Patch& warpedPatch, Landmark& landmark, Frame& sourceFrame, Frame& targetFrame, QDVO::Graph& g, const int patchWidth = PATCH_RADIUS);
};
}

#endif // PATCHWARPER_H
