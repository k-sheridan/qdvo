
.. _program_listing_file_src_CameraModels_EquidistantCameraModel.h:

Program Listing for File EquidistantCameraModel.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_CameraModels_EquidistantCameraModel.h>` (``src/CameraModels/EquidistantCameraModel.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef EQUIDISTANTCAMERAMODEL_H
   #define EQUIDISTANTCAMERAMODEL_H
   
   #include "CameraModel.hpp"
   #include "GlobalDefinitions.h"
   #include <vector>
   #include <Eigen/LU>
   
   namespace QDVO {
   class EquidistantCameraModel : public CameraModel
   {
   public:
   
       EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height, const Eigen::Vector4d& distortionCoeffs);
   
       QDVO::Result<QDVO::Vector2> project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian = nullptr);
   
       QDVO::Result<QDVO::Vector3> unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian = nullptr);
   
       SCALAR_TYPE distortionFn(SCALAR_TYPE theta)
       {
           // do this inline for both performance and ease.
           const SCALAR_TYPE theta2 = theta*theta;
           SCALAR_TYPE thetaAccumulated = theta; // used to accumulate the
   
           SCALAR_TYPE result = thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += this->distortionCoeffs(0)*thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += this->distortionCoeffs(1)*thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += this->distortionCoeffs(2)*thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += this->distortionCoeffs(3)*thetaAccumulated;
   
           return result;
       }
   
       SCALAR_TYPE distortionFnDerivative(SCALAR_TYPE theta)
       {
           // do this inline for both performance and ease.
           const SCALAR_TYPE theta2 = theta*theta;
           SCALAR_TYPE thetaAccumulated = theta2; // used to accumulate the
   
           SCALAR_TYPE result = 1;
   
           result += 3*this->distortionCoeffs(0)*thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += 5*this->distortionCoeffs(1)*thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += 7*this->distortionCoeffs(2)*thetaAccumulated;
   
           thetaAccumulated *= theta2;
           result += 9*this->distortionCoeffs(3)*thetaAccumulated;
   
           return result;
       }
   
   private:
       Eigen::Vector4d distortionCoeffs; // the 3rd, 5th, 7th, and 9th order coefficients of a polynomial function of the landmark angle. Same as used in Kalibr.
   
       struct UniformRadiusLookUpTable {
           std::vector<SCALAR_TYPE> umap; // r = f(\theta) = umap(\theta / resolution)
           double resolution = EQUIDISTANT_CAMERA_MODEL_RADIUS_MAP_RESOLUTION; // theta = index*resolution
       } radiusLookUpTable;
   
   
   };
   }
   
   #endif // EQUIDISTANTCAMERAMODEL_H
