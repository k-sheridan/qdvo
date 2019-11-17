
.. _program_listing_file_src_CameraModels_FOVCameraModel.hpp:

Program Listing for File FOVCameraModel.hpp
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_CameraModels_FOVCameraModel.hpp>` (``src/CameraModels/FOVCameraModel.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef FOVCAMERAMODEL_H
   #define FOVCAMERAMODEL_H
   
   #include "CameraModel.hpp"
   
   namespace  QDVO {
   class FOVCameraModel : public CameraModel
   {
   public:
       FOVCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height);
   };
   }
   
   #endif // FOVCAMERAMODEL_H
