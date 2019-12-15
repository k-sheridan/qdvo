
.. _program_listing_file_src_Pipeline_BasicPipeline.h:

Program Listing for File BasicPipeline.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Pipeline_BasicPipeline.h>` (``src/Pipeline/BasicPipeline.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "GlobalDefinitions.h"
   #include "DataStructures/Graph.h"
   #include "FrontFndVisualOdometry.h"
   #include "SlidingWindowEstimator.h"
   #include "PatchComparer.h"
   #include "PatchWarper.h"
   #include "DataStructures/RadialSearchPattern.h"
   #include <opencv2/core.hpp>
   #include <opencv2/highgui.hpp>
   #include "Types.h"
   #include "Optimizer/ParallelAlgorithms/ParallelAlgorithms.h"
   
   namespace QDVO
   {
   
   class BasicPipeline
   {
   public:
       BasicPipeline();
   
       Graph graph;
   
       FrontEndVisualOdometry frontEndVisualOdometry;
   
       SlidingWindowEstimator swe; 
   
       void initialize();
   
       void addFrame(cv::Mat &image, const double &time, const ID_TYPE cameraID = 1);
   
       void addCamera(std::unique_ptr<QDVO::CameraModel> &cameraModel, const ID_TYPE cameraID = 1);
   
       void createNewLandmarks(std::unique_ptr<QDVO::Frame> &keyframe, std::unique_ptr<QDVO::FeatureDetector> &featureDetector);
   
       void activateNewLandmarks();
   
       void initializeCorrespondenceDistributionsForCurrentFrame();
   
       void runEpipolarDepthEstimators();
   
       void runMarginalizationStrategy();
   
       bool isCurrentFrameAKeyframe();
   
       void updatePatchComparers();
   
       // -=-=-=-===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
       // ZNCC IMPLEMENTATION SPECIFIC and PREALLOCATED / PRECOMPUTED DATA
   
       std::unique_ptr<QDVO::FeatureDetector> featureDetector;
   
       std::unordered_map<ID_TYPE, std::shared_ptr<PatchComparer>> patchComparers;
   
       std::shared_ptr<RadialSearchPattern> radialSearchPatternPtr;
   
       std::unique_ptr<QDVO::PatchWarper> patchWarper;
   };
   
   } // namespace QDVO
   
