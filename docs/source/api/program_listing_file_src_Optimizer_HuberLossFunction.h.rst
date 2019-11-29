
.. _program_listing_file_src_Optimizer_HuberLossFunction.h:

Program Listing for File HuberLossFunction.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_HuberLossFunction.h>` (``src/Optimizer/HuberLossFunction.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   namespace ArgMin {
   template <typename ScalarType>
   class HuberLossFunction
   {
   public:
   
       HuberLossFunction(ScalarType c_) {
           c = c_;
       } 
   
       ScalarType c;
   
       ScalarType computeWeight(ScalarType errorNorm, ScalarType errorNormSquared)
       {
           if (errorNorm < c) {
               return 1.0;
           } else {
               return c / errorNorm;
           }
       }
   };
   
   } // namespace ArgMin
