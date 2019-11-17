
.. _program_listing_file_src_Estimation_SlidingWindowEstimator.h:

Program Listing for File SlidingWindowEstimator.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Estimation_SlidingWindowEstimator.h>` (``src/Estimation/SlidingWindowEstimator.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef SLIDINGWINDOWESTIMATOR_H
   #define SLIDINGWINDOWESTIMATOR_H
   
   #include "DataStructures/Graph.h"
   
   class SlidingWindowEstimator
   {
   public:
       SlidingWindowEstimator();
   
       void run(QDVO::Graph& graph);
   
       void removeOutliers(QDVO::Graph& graph);
   
       void runMarginalizationStrategy(QDVO::Graph& graph);
   };
   
   #endif // SLIDINGWINDOWESTIMATOR_H
