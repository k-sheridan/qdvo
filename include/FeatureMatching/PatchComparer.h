#ifndef PATCHCOMPARER_H
#define PATCHCOMPARER_H

namespace QDVO {
/*
 * base class used to compare two image patches.
 * For speed, this is implemented in a way which will compare a patch with a pixel, image combinination.
 * This base class is implemented as a ZNCC.
 */
class PatchComparer
{
public:
    PatchComparer();
};
}

#endif // PATCHCOMPARER_H
