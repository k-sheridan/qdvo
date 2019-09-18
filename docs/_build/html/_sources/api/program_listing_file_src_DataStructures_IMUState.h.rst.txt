
.. _program_listing_file_src_DataStructures_IMUState.h:

Program Listing for File IMUState.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_IMUState.h>` (``src/DataStructures/IMUState.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <sophus/common.hpp>
   #include <sophus/types.hpp>
   #include <sophus/se3.hpp>
   #include "GlobalDefinitions.h"
   #include "Types.h"
   
   namespace  QDVO {
   class IMUState
   {
   public:
       IMUState();
   
       int dimensions = 9;
   
       double time; // the time of this state.
   
       QDVO::SO3 attitude;
       QDVO::Vector3 pos;
       QDVO::Vector3 vel;
       // Optionally, I may need angular velocity for the tightly coupled quadrotor integration.
   
   
       void update(const Eigen::Matrix<SCALAR_TYPE, 9, 1>& dx);
       void updatePose(const Eigen::Matrix<SCALAR_TYPE, 6, 1> &dx);
       void updateVelocity(const Eigen::Matrix<SCALAR_TYPE, 3, 1> &dx);
   
       QDVO::SE3 getSE3();
   };
   }
   
