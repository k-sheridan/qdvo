
.. _program_listing_file_src_FeatureMatching_PatchComparer.h:

Program Listing for File PatchComparer.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_FeatureMatching_PatchComparer.h>` (``src/FeatureMatching/PatchComparer.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef PATCHCOMPARER_H
   #define PATCHCOMPARER_H
   
   #include "GlobalDefinitions.h"
   #include <opencv2/core.hpp>
   #include <opencv2/imgproc.hpp>
   #include "Frame.h"
   #include "Patch.h"
   #include "ImageStatisticsLUT.h"
   #include "Types.h"
   #include <Eigen/Core>
   
   namespace QDVO {
   
   /*
    * base class used to compare two image patches.
    * For speed, this is implemented in a way which will compare a patch with a pixel, image combinination.
    * This base class is implemented as a ZNCC.
    */
   class PatchComparer
   {
   public:
   
       ImageStatisticsLUT meanStdDevTable; // precomputed LUT used to speed up the ZNCC or ZNSSD.
   
       PatchComparer();
   
       /*
        * uses the zero mean normalized cross correlation to determine how well a target frame pixel matches the patch.
        * result is bounded on the interval [0, 1] where 0 is a highly unlikely match, and 1 is a highly likely match.
        *
        * no out of bounds checks are performed at this level.
        */
       virtual QDVO::Result<SCALAR_TYPE> compare(QDVO::Patch& templatePatch, Frame& targetFrame, Eigen::Vector2i& pixel);
   
   
   };
   }
   
   #endif // PATCHCOMPARER_H
