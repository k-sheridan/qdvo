#include "PatchWarper.h"

QDVO::PatchWarper::PatchWarper()
{

}

void QDVO::PatchWarper::warpPatchToTargetFrame(Patch& warpedPatch, Landmark& landmark, Frame& sourceFrame, Frame& targetFrame, QDVO::Graph& g, const int patchWidth)
{
    QDVO::SE3 T_w_sourceImu = sourceFrame.imustate.getSE3();
    QDVO::SE3 T_w_targetImu = targetFrame.imustate.getSE3();

    QDVO::SE3 T_i_sourceCam = g.getExtrinsic(sourceFrame.camID);
    QDVO::SE3 T_i_targetCam = g.getExtrinsic(targetFrame.camID);

    QDVO::SE3 T_sourceCam_targetCam = (T_w_sourceImu * T_i_sourceCam).inverse() * (T_w_targetImu * T_i_targetCam);
    QDVO::SE3 T_targetCam_sourceCam = T_sourceCam_targetCam.inverse();

    assert("not ready yet" && false);
}
