
.. _program_listing_file_src_Estimation_SlidingWindowEstimator.cpp:

Program Listing for File SlidingWindowEstimator.cpp
===================================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Estimation_SlidingWindowEstimator.cpp>` (``src/Estimation/SlidingWindowEstimator.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "SlidingWindowEstimator.h"
   
   using namespace QDVO;
   
   SlidingWindowEstimator::SlidingWindowEstimator()
   {
       // verify this compiles.
       optimizer.prior.A0.getRowMap<ArgMin::SE3>().begin()->second.getVariableMap<ArgMin::SE3>().begin();
   }
   
   
   void SlidingWindowEstimator::run(QDVO::Graph& graph)
   {
   
   }
   
   void SlidingWindowEstimator::removeOutliers(QDVO::Graph& graph)
   {
   
   }
   
   void SlidingWindowEstimator::runMarginalizationStrategy(QDVO::Graph& graph)
   {
   
   }
