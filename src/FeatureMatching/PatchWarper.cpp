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

    QDVO::Vector3 pt_source = landmark.getEuclideanPoint();
    QDVO::Vector3 r_pt_target = T_sourceCam_targetCam.translation() - pt_source;

    QDVO::Vector3 normal = (r_pt_target / r_pt_target.norm()) - (pt_source / pt_source.norm());
    normal.normalize();

    QDVO::Vector3 n = T_targetCam_sourceCam.so3() * normal;
    QDVO::Vector3 p0 = T_targetCam_sourceCam * landmark.getEuclideanPoint();

    if (p0(2) <= 1e-10)
    {
        throw std::runtime_error("failed to warp patch. point behind camera.");
    }

    QDVO::Vector3 u0 = p0/p0(2);
    Eigen::Matrix<SCALAR_TYPE, 2, 2> projJac;

    QDVO::Vector2 px0 = targetFrame.cm->project(u0, &projJac);

    Eigen::Matrix<SCALAR_TYPE, 2, 2> unprojJac = projJac.inverse();

    int patchDim = 2*patchWidth + 1;
    warpedPatch.image = cv::Mat(patchDim, patchDim, CV_8U);
    warpedPatch.level = 0;

    QDVO::Vector3 u, p;
    u(2) = 1;
    for (int deltaX = -patchWidth; deltaX <= patchWidth; ++deltaX)
    {
        for (int deltaY = -patchWidth; deltaY <= patchWidth; ++deltaY)
        {
            u.block(0, 0, 2, 1) = u0.block(0, 0, 2, 1) + unprojJac * QDVO::Vector2(deltaX, deltaY);
            p = T_sourceCam_targetCam * (((p0.dot(n)) / (u.dot(n))) * u);

            QDVO::Vector2 px = sourceFrame.cm->project(p);
            float brightness = sourceFrame.imagePyr.getColorSubpix(sourceFrame.imagePyr.getImage(), cv::Point2f(px(0), px(1)));

            warpedPatch.image.at<uint8_t>(deltaY + patchWidth, deltaX + patchWidth) = uint8_t(brightness);
        }
    }
}
