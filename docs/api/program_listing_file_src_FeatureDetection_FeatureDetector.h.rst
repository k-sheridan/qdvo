
.. _program_listing_file_src_FeatureDetection_FeatureDetector.h:

Program Listing for File FeatureDetector.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_FeatureDetection_FeatureDetector.h>` (``src/FeatureDetection/FeatureDetector.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "GlobalDefinitions.h"
   #include <opencv2/core.hpp>
   #include <opencv2/imgproc.hpp>
   #include "Feature.h"
   #include "Frame.h"
   #include <algorithm>
   
   
   // Feature Detector Settings
   #define N_SECTIONS 100
   #define HARRIS_K  0.05
   #define HARRIS_WIDTH 3
   #define EDGE_WEIGHT 0.1
   #define INVARIANT_THRESHOLD  0.5
   #define MINIMUM_NORMALIZED_GRADIENT_MAG 0.120
   
   #define USE_SPATIAL_MASK true
   #define SPATIAL_MASK_RADIUS 6
   
   namespace  QDVO {
   class FeatureDetector
   {
   public:
       FeatureDetector();
   
       std::vector<QDVO::Feature> detectFeatures(Frame& frame, const int level = 0);
   
       struct FeatureCandidate{
           int x, y; // pixel position
           SCALAR_TYPE dxdx, dxdy, dydy; // structure tensor information
           SCALAR_TYPE det, trace; // determinant and trace of the structure tensor.
           SCALAR_TYPE gradientNorm;
           SCALAR_TYPE harris;
           SCALAR_TYPE score = 0; // stores the feature score which I have described in my paper.
       };
   
   private:
       cv::Mat dx, dy; // preallocated containers for the image gradients.
       cv::Mat dxdx, dydy, dxdy; // preallocated containers for structure tensors.
   
       cv::Mat spatialMask; // used to ensure no two features are too close to each other.
   };
   }
   
