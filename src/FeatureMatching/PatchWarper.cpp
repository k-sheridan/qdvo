#include "PatchWarper.h"

#include "Logging.h"

QDVO::PatchWarper::PatchWarper() {}

void QDVO::PatchWarper::warpPatchToTargetFrame(
    QDVO::Result<Patch> &warpedPatch, Landmark &landmark, Frame &sourceFrame,
    Frame &targetFrame, QDVO::Graph &graph, const int patchRadius, int level) {
  warpedPatch.reset();  // reset the patch.

  QDVO::SE3 T_w_sourceImu = sourceFrame.imustate.getSE3();
  QDVO::SE3 T_w_targetImu = targetFrame.imustate.getSE3();

  QDVO::SE3 &T_i_sourceCam =
      *graph.getExtrinsicMap().at(sourceFrame.extrinsicKey);
  QDVO::SE3 &T_i_targetCam =
      *graph.getExtrinsicMap().at(targetFrame.extrinsicKey);

  QDVO::SE3 T_sourceCam_targetCam = (T_w_sourceImu * T_i_sourceCam).inverse() *
                                    (T_w_targetImu * T_i_targetCam);
  QDVO::SE3 T_targetCam_sourceCam = T_sourceCam_targetCam.inverse();

  QDVO::Vector3 pt_source = landmark.getEuclideanPoint();
  QDVO::Vector3 r_pt_target = T_sourceCam_targetCam.translation() - pt_source;

  QDVO::Vector3 normal =
      (r_pt_target / r_pt_target.norm()) - (pt_source / pt_source.norm());
  normal.normalize();

  QDVO::Vector3 n = T_targetCam_sourceCam.so3() * normal;
  QDVO::Vector3 p0 = T_targetCam_sourceCam * landmark.getEuclideanPoint();

  if (p0(2) <= 1e-10) {
    LOG_TRACE(
        "Could not warp patch because the associated landmark is behind the "
        "target frame. pt_target: \n{}\n",
        p0);
    return;
  }

  QDVO::Vector3 u0 = p0 / p0(2);
  Eigen::Matrix<SCALAR_TYPE, 2, 2> projJac;

  auto px0 = graph.getCameraModelMap()
                 .at(targetFrame.cameraModelKey)
                 ->first->project(u0, &projJac);
  if (!px0.has_value()) {
    LOG_TRACE(
        "Could not warp patch because landmark was not projectable into the "
        "target frame.");
    return;
  }

  Eigen::Matrix<SCALAR_TYPE, 2, 2> unprojJac = projJac.inverse();
  assert(patchRadius == PATCH_RADIUS);
  Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH> imageData;

  auto px_source = graph.getCameraModelMap()
                       .at(sourceFrame.cameraModelKey)
                       ->first->project(pt_source, &projJac);
  if (!px_source.has_value()) {
    LOG_ERROR(
        "Could not warp patch because landmark was not projectable into the "
        "source frame.");
    return;
  }

  // Compute the dimension ratios.
  SCALAR_TYPE sourceWidthRatio =
      ((SCALAR_TYPE)(sourceFrame.imagePyr.getImage(level).cols())) /
      ((SCALAR_TYPE)(sourceFrame.imagePyr.getImage(0).cols()));
  SCALAR_TYPE sourceHeightRatio =
      ((SCALAR_TYPE)(sourceFrame.imagePyr.getImage(level).rows())) /
      ((SCALAR_TYPE)(sourceFrame.imagePyr.getImage(0).rows()));

  SCALAR_TYPE targetWidthRatio =
      ((SCALAR_TYPE)(targetFrame.imagePyr.getImage(level).cols())) /
      ((SCALAR_TYPE)(targetFrame.imagePyr.getImage(0).cols()));
  SCALAR_TYPE targetHeightRatio =
      ((SCALAR_TYPE)(targetFrame.imagePyr.getImage(level).rows())) /
      ((SCALAR_TYPE)(targetFrame.imagePyr.getImage(0).rows()));

  QDVO::Vector3 u, p;
  u(2) = 1;

  for (int deltaX = -patchRadius; deltaX <= patchRadius; ++deltaX) {
    for (int deltaY = -patchRadius; deltaY <= patchRadius; ++deltaY) {
      u.block(0, 0, 2, 1) =
          u0.block(0, 0, 2, 1) +
          unprojJac * QDVO::Vector2(deltaX / targetWidthRatio,
                                    deltaY / targetHeightRatio);
      p = T_sourceCam_targetCam * (((p0.dot(n)) / (u.dot(n))) * u);

      QDVO::Vector2 px =
          px_source.value() + projJac * ((p.block(0, 0, 2, 1) / p(2)) -
                                         landmark.bearing.block(0, 0, 2, 1));

      px(0) *= sourceWidthRatio;
      px(1) *= sourceHeightRatio;

      auto brightness =
          sourceFrame.imagePyr.getImage(level).getSubPixelIntensity(px);
      if (!brightness.has_value()) {
        LOG_TRACE(
            "Could not warp patch. It was not possible to sample the source "
            "image.");
        return;
      }

      imageData(deltaY + patchRadius, deltaX + patchRadius) =
          (brightness.value());
    }
  }

  warpedPatch = QDVO::Patch(imageData, level);
}
