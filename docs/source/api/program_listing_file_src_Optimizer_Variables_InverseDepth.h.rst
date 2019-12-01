
.. _program_listing_file_src_Optimizer_Variables_InverseDepth.h:

Program Listing for File InverseDepth.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Variables_InverseDepth.h>` (``src/Optimizer/Variables/InverseDepth.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Optimizer/OptimizableVariable.h"
   #include <limits>
   
   namespace ArgMin
   {
   
   class InverseDepth : public ArgMin::OptimizableVariable<double, 1>
   {
   public:
       double value;
   
       InverseDepth() = default;
   
       InverseDepth(double dinv) : value(dinv) {}
   
       void update(const Eigen::Matrix<double, 1, 1> &dx)
       {
           // The inverse depth must come in as a valid 
           assert(value >= 0 && value <= std::numeric_limits<double>::max());
           value += dx(0,0);
   
           if (value < 0) {
               value = std::numeric_limits<double>::min();
           }
       }
   };
   
   } // namespace ArgMin
