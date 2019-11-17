
.. _program_listing_file_src_Optimizer_Variables_InverseDepth.h:

Program Listing for File InverseDepth.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Variables_InverseDepth.h>` (``src/Optimizer/Variables/InverseDepth.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Optimizer/OptimizableVariable.h"
   
   namespace ArgMin
   {
   
   class InverseDepth : public ArgMin::OptimizableVariable<double, 1>
   {
   public:
       InverseDepth()
       {
       }
   
       void update(const Eigen::Matrix<double, 1, 1> &dx)
       {
       }
   };
   
   } // namespace ArgMin
