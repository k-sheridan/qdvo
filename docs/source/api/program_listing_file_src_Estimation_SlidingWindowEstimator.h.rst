
.. _program_listing_file_src_Estimation_SlidingWindowEstimator.h:

Program Listing for File SlidingWindowEstimator.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Estimation_SlidingWindowEstimator.h>` (``src/Estimation/SlidingWindowEstimator.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "DataStructures/Graph.h"
   #include "Optimizer/SSEOptimizer.h"
   #include "Optimizer/Variables/SE3.h"
   #include "Optimizer/MetaHelpers.h"
   #include "Optimizer/Key.h"
   #include "Optimizer/Containers.h"
   
   namespace QDVO {
   
   class SlidingWindowEstimator
   {
   public:
       SlidingWindowEstimator();
   
       void run(QDVO::Graph& graph);
   
       void removeOutliers(QDVO::Graph& graph);
   
       void runMarginalizationStrategy(QDVO::Graph& graph);
   
       ArgMin::SSEOptimizer<ArgMin::Scalar<double>, ArgMin::VariableGroup<ArgMin::SE3>, ArgMin::ErrorTermGroup<ArgMin::SE3>> optimizer;
   };
   
   } // namespace QDVO
