
.. _program_listing_file_src_Estimation_EpipolarDepthEstimator.h:

Program Listing for File EpipolarDepthEstimator.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Estimation_EpipolarDepthEstimator.h>` (``src/Estimation/EpipolarDepthEstimator.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   namespace QDVO {
   
   class EpipolarDepthEstimator
   {
   public:
       EpipolarDepthEstimator();
   
       bool initialized = false; // flag which means that the landmark depth has been succesfully initialized.
   };
   
   } // namespace QDVO
