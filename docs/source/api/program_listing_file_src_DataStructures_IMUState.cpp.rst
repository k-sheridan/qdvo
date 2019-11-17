
.. _program_listing_file_src_DataStructures_IMUState.cpp:

Program Listing for File IMUState.cpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_IMUState.cpp>` (``src/DataStructures/IMUState.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "IMUState.h"
   
   QDVO::IMUState::IMUState()
   {
   
   }
   
   void QDVO::IMUState::update(const Eigen::Matrix<SCALAR_TYPE, 9, 1> &dx)
   {
       this->updatePose(dx.block(0, 0, 6, 1));
       this->updateVelocity(dx.block(6, 0, 3, 1));
   }
   
   void QDVO::IMUState::updatePose(const Eigen::Matrix<SCALAR_TYPE, 6, 1> &dx)
   {
       this->pos += dx.block(0, 0, 3, 1);
       this->attitude *= Sophus::SO3<SCALAR_TYPE>(dx.block(3, 0, 3, 1));
   }
   
   void QDVO::IMUState::updateVelocity(const Eigen::Matrix<SCALAR_TYPE, 3, 1> &dx)
   {
       this->vel += dx.block(6, 0, 3, 1);
   }
   
   QDVO::SE3 QDVO::IMUState::getSE3()
   {
       return Sophus::SE3<SCALAR_TYPE>(this->attitude.unit_quaternion(), this->pos);
   }
   
   
   
   
   
