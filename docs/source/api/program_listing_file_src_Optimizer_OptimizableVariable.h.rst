
.. _program_listing_file_src_Optimizer_OptimizableVariable.h:

Program Listing for File OptimizableVariable.h
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_OptimizableVariable.h>` (``src/Optimizer/OptimizableVariable.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <Eigen/Core>
   
   namespace ArgMin {
   
   template <typename ScalarType, size_t Dimension>
   class OptimizableVariable {
       public:
       typedef ScalarType scalar_type;
       static const size_t dimension = Dimension;
   
   };
   
   } // namespace QDVO
