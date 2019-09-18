
.. _program_listing_file_src_FeatureMatching_PatchWarper.cpp:

Program Listing for File PatchWarper.cpp
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_FeatureMatching_PatchWarper.cpp>` (``src/FeatureMatching/PatchWarper.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "PatchWarper.h"
   
   QDVO::PatchWarper::PatchWarper()
   {
   }
   
   void QDVO::PatchWarper::warpPatchToTargetFrame(QDVO::Result<Patch> &warpedPatch, Landmark &landmark, Frame &sourceFrame, Frame &targetFrame, QDVO::Graph &g, const int patchRadius)
   {
       warpedPatch.reset(); // reset the patch.
   
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
           //throw std::runtime_error("failed to warp patch. point behind camera.");
           return;
       }
   
       QDVO::Vector3 u0 = p0 / p0(2);
       Eigen::Matrix<SCALAR_TYPE, 2, 2> projJac;
   
       auto px0 = targetFrame.cm->project(u0, &projJac);
       if (!px0.has_value())
       {
           return;
       }
       auto px_source = sourceFrame.cm->project(pt_source, &projJac);
       if (!px_source.has_value())
       {
           return;
       }
   
       Eigen::Matrix<SCALAR_TYPE, 2, 2> unprojJac = projJac.inverse();
       assert(patchRadius == PATCH_RADIUS);
       Eigen::Matrix<float, PATCH_WIDTH, PATCH_WIDTH> imageData;
   
       QDVO::Vector3 u, p;
       u(2) = 1;
       for (int deltaX = -patchRadius; deltaX <= patchRadius; ++deltaX)
       {
           for (int deltaY = -patchRadius; deltaY <= patchRadius; ++deltaY)
           {
               u.block(0, 0, 2, 1) = u0.block(0, 0, 2, 1) + unprojJac * QDVO::Vector2(deltaX, deltaY);
               p = T_sourceCam_targetCam * (((p0.dot(n)) / (u.dot(n))) * u);
   
               QDVO::Vector2 px = px_source.value() + projJac * ((p.block(0, 0, 2, 1) / p(2)) - landmark.bearing.block(0, 0, 2, 1));
   
               auto brightness = sourceFrame.imagePyr.getImage().getSubPixelIntensity(px);
               if (!brightness.has_value())
               {
                   return;
               }
   
               imageData(deltaY + patchRadius, deltaX + patchRadius) = (brightness.value());
           }
       }
   
       warpedPatch = QDVO::Patch(imageData);
   }
