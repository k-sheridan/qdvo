
.. _program_listing_file_src_ErrorTerms_QuasiDirectErrorTerm.h:

Program Listing for File QuasiDirectErrorTerm.h
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_ErrorTerms_QuasiDirectErrorTerm.h>` (``src/ErrorTerms/QuasiDirectErrorTerm.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Types.h"
   #include "CorrespondenceDistribution.h"
   #include "CameraModel.h"
   
   namespace QDVO
   {
   
   class QuasiDirectFactor : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, QDVO::InverseDepth>
   {
   
   private:
       QDVO::CorrespondenceDistribution *cdPtr; // Do not delete.
       QDVO::Vector3 landmarkBearing;           // The bearing to the landmark in the landmark frame.
       QDVO::CameraModel* cmPtr_obsFrame; // observation frame camera model interface. 
   
   public:
       QuasiDirectFactor(gtsam::Key obsFrameKey,
                         gtsam::Key landmarkFrameKey,
                         gtsam::Key landmarkInverseDepthKey,
                         QDVO::CorrespondenceDistribution &correspondenceDistribution,
                         QDVO::Vector3 &landmarkBearing,
                         QDVO::CameraModel* obsFrameCameraModel,
                         gtsam::SharedNoiseModel model) : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, QDVO::InverseDepth>(model, obsFrameKey, landmarkFrameKey, landmarkInverseDepthKey)
       {
           this->cdPtr = &correspondenceDistribution;
           this->landmarkBearing = landmarkBearing;
           this->cmPtr_obsFrame = obsFrameCameraModel;
       }
   
       gtsam::Vector evaluateError(const gtsam::Pose3 &obsFrame,
                                   const gtsam::Pose3 &landmarkFrame,
                                   const QDVO::InverseDepth &landmarkInverseDepth,
                                   boost::optional<gtsam::Matrix &> H1 = boost::none,
                                   boost::optional<gtsam::Matrix &> H2 = boost::none,
                                   boost::optional<gtsam::Matrix &> H3 = boost::none)
       {
           const auto& A = obsFrame.rotation();
           const auto& B = landmarkFrame.rotation();
   
           const auto&  d = obsFrame.translation();
           const auto&  e = landmarkFrame.translation();
   
           const auto& u0 = landmarkBearing;
           const auto& dinv = landmarkInverseDepth.dinv;
   
           const auto p_obs = A..transpose()*B*u0*(1/dinv) + A.transpose()*(e - d);
   
           QDVO::Matrix2 projJac;
           
   
       }
   };
   
   } //namespace QDVO
