
.. _program_listing_file_src_CameraModels_FOVCameraModel.cpp:

Program Listing for File FOVCameraModel.cpp
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_CameraModels_FOVCameraModel.cpp>` (``src/CameraModels/FOVCameraModel.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "FOVCameraModel.hpp"
   
   QDVO::FOVCameraModel::FOVCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height) :
       QDVO::CameraModel (fx, fy, cx, cy, fov, width, height)
   {
   
   }
