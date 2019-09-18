
.. _program_listing_file_src_Settings.h:

Program Listing for File Settings.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Settings.h>` (``src/Settings.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "GlobalDefinitions.h"
   namespace QDVO {
   
   class Settings
   {
   public:
   
       // global settings file
       int nKeyframes = 7;
   
       // feature detection settings
       struct FeatureDetectionSettings {
           int nFeaturesDesired = 400;
           int nSections = 100;
           SCALAR_TYPE harrisK = 0.05;
           SCALAR_TYPE edgeWeight = 0.1;
           SCALAR_TYPE invariantThreshold = 0.1;
           SCALAR_TYPE minimumNormalizedGradientMagnitude = 0.0306;
       } feature_detection;
   };
   }
   
