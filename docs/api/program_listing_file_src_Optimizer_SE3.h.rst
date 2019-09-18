
.. _program_listing_file_src_Optimizer_SE3.h:

Program Listing for File SE3.h
==============================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_SE3.h>` (``src/Optimizer/SE3.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "OptimizableVariable.h"
   
   namespace LittleOptimizer {
   
   class SE3 : public OptimizableVariable<6> {
   
   };
   
   } // namespace LittleOptimizer
