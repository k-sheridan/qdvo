
.. _program_listing_file_src_Optimizer_PSDLinearSystem.h:

Program Listing for File PSDLinearSystem.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_PSDLinearSystem.h>` (``src/Optimizer/PSDLinearSystem.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "MetaHelpers.h"
   #include "SparseBlockMatrix.h"
   
   namespace LittleOptimizer {
   
   template <typename... T>
   class PSDLinearSystem;
   
   template <typename ScalarType, typename... Variables>
   class PSDLinearSystem<Scalar<ScalarType>, VariableGroup<Variables...>> {
   
   };
   
   }
