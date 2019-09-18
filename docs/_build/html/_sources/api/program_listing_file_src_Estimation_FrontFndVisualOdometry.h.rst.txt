
.. _program_listing_file_src_Estimation_FrontFndVisualOdometry.h:

Program Listing for File FrontFndVisualOdometry.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Estimation_FrontFndVisualOdometry.h>` (``src/Estimation/FrontFndVisualOdometry.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Graph.h"
   #include "LittleOptimizer.h"
   
   class FrontEndVisualOdometry
   {
   public:
       FrontEndVisualOdometry();
   
       void run(QDVO::Graph& graph);
   };
   
