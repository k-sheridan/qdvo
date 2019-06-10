#ifndef PATCHWARPER_H
#define PATCHWARPER_H

#include <GlobalDefinitions.h>
#include <opencv2/core.hpp>
#include <Frame.h>
#include <Patch.h>

namespace QDVO {
/*
 * This class is used to warp a feature represented as a patch into a new frame.
 * the patch normal is assumed to be oriented towards the center of the baseline between the source and target frame.
 */
class PatchWarper
{
public:
    PatchWarper();

    SCALAR_TYPE compare(const Patch& patch, const Frame& targetFrame);
};
}

#endif // PATCHWARPER_H
