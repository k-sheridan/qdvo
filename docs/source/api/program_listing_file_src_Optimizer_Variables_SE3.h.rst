
.. _program_listing_file_src_Optimizer_Variables_SE3.h:

Program Listing for File SE3.h
==============================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Variables_SE3.h>` (``src/Optimizer/Variables/SE3.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Optimizer/OptimizableVariable.h"
   #include <sophus/se3.hpp>
   
   namespace ArgMin
   {
   
   class SE3 : public ArgMin::OptimizableVariable<double, 6>
   {
   public:
       Sophus::SE3<double> value;
   
       SE3() = default;
   
       SE3(const Sophus::SE3<double>& se3) : value(se3) {}
   
       SE3(Sophus::SE3<double> se3) : value(se3) {}
   
       void update(const Eigen::Matrix<double, 6, 1> &dx)
       {
           value.so3() *= Sophus::SO3d::exp(dx.block<3, 1>(0, 0));
           value.translation() += dx.block<3, 1>(3, 0);
       }
   };
   
   } // namespace ArgMin
