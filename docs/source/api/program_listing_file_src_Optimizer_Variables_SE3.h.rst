
.. _program_listing_file_src_Optimizer_Variables_SE3.h:

Program Listing for File SE3.h
==============================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_Variables_SE3.h>` (``src/Optimizer/Variables/SE3.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "Optimizer/OptimizableVariable.h"
   
   class SE3 : public ArgMin::OptimizableVariable<6> {
   
   };
