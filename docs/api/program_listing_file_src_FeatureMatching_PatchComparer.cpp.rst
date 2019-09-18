
.. _program_listing_file_src_FeatureMatching_PatchComparer.cpp:

Program Listing for File PatchComparer.cpp
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_FeatureMatching_PatchComparer.cpp>` (``src/FeatureMatching/PatchComparer.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "PatchComparer.h"
   
   QDVO::PatchComparer::PatchComparer(){
   
   }
   
   QDVO::Result<SCALAR_TYPE> QDVO::PatchComparer::compare(QDVO::Patch& patch, Frame& targetFrame, Eigen::Vector2i& pixel)
   {
       cv::Rect roi(cv::Point2i(pixel(0), pixel(1)) - cv::Point2i(PATCH_RADIUS, PATCH_RADIUS), cv::Size2i(PATCH_WIDTH, PATCH_WIDTH));
   
       Eigen::Vector2i shift(PATCH_RADIUS, PATCH_RADIUS);
       Eigen::Vector2i tl = pixel - shift;
       Eigen::Vector2i br = pixel + shift;
   
       assert(this->meanStdDevTable.framePtr == &targetFrame);
       assert(patch.getStdDev() > 1e-8);
       assert(PATCH_WIDTH == 2*PATCH_RADIUS + 1);
   
       QDVO::ImageType& targetImage = targetFrame.imagePyr.getImage().getImageData();
   
       if (tl(0) < 0 || tl(1) < 0 || br(0) >= targetImage.cols() || br(1) >= targetImage.rows())
       {
           return {};
       }
   
       // Get the approximate mean of the test patch
       //float approxMean = this->meanStdDevTable.getMean(cv::Point2i(pixel(0), pixel(1)));
   
       // get the test patch from the image and compute its zero mean self.
       Eigen::Matrix<QDVO::ImageIntensityType, PATCH_WIDTH, PATCH_WIDTH> patchData = targetImage.block<PATCH_WIDTH, PATCH_WIDTH>(tl(1), tl(0));
       QDVO::Patch testPatch = QDVO::Patch(patchData);
      
       SCALAR_TYPE resultingScore = (patch.getZeroMeanImageMatrix().array() * testPatch.getZeroMeanImageMatrix().array()).sum()
       / sqrt(patch.getSumZeroMeanSquared() * testPatch.getSumZeroMeanSquared());
        resultingScore = ((resultingScore + 1) / 2);
   
       return resultingScore;
   }
