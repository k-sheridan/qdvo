
.. _program_listing_file_src_FeatureMatching_PatchWarper.h:

Program Listing for File PatchWarper.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_FeatureMatching_PatchWarper.h>` (``src/FeatureMatching/PatchWarper.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef PATCHWARPER_H
   #define PATCHWARPER_H
   
   #include "GlobalDefinitions.h"
   #include <opencv2/core.hpp>
   #include "DataStructures/Frame.h"
   #include "DataStructures/Patch.h"
   #include "DataStructures/Graph.h"
   #include "Types.h"
   
   namespace QDVO {
   /*
    * This class is used to warp a feature represented as a patch into a new frame.
    * the patch normal is assumed to be oriented towards the center of the baseline between the source and target frame.
    */
   class PatchWarper
   {
   public:
       PatchWarper();
   
       /*
        * computes a warped patch in the target frame assuming the feature lies on a flat surface.
        */
       void warpPatchToTargetFrame(QDVO::Result<Patch>& warpedPatch, Landmark& landmark, Frame& sourceFrame, Frame& targetFrame, QDVO::Graph& g, const int patchWidth = PATCH_RADIUS);
   };
   }
   
   #endif // PATCHWARPER_H
