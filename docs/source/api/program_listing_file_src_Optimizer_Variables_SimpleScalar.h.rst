
.. _program_listing_file_src_Optimizer_Variables_SimpleScalar.h:

Program Listing for File SimpleScalar.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Variables_SimpleScalar.h>` (``src/Optimizer/Variables/SimpleScalar.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Optimizer/OptimizableVariable.h"
   
   namespace ArgMin
   {
   
   class SimpleScalar : public ArgMin::OptimizableVariable<double, 1>
   {
   public:
   
       double value;
   
       SimpleScalar() = default;
   
       SimpleScalar(double val) : value(val)
       {
       }
   
       void update(const Eigen::Matrix<double, 1, 1> &dx)
       {
           value += dx(0, 0);
       }
   };
   
   } // namespace ArgMin
