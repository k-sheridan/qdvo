
.. _program_listing_file_src_DataStructures_Landmark.cpp:

Program Listing for File Landmark.cpp
=====================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_Landmark.cpp>` (``src/DataStructures/Landmark.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "Landmark.h"
   
   QDVO::Landmark::Landmark()
   {
       this->status = QDVO::Landmark::LandmarkStatus::INACTIVE;
   }
   
   Eigen::Matrix<SCALAR_TYPE, 3, 1> QDVO::Landmark::getEuclideanPoint()
   {
       assert(dinv > std::numeric_limits<SCALAR_TYPE>::min());
   
       return this->bearing / this->dinv;
   }
