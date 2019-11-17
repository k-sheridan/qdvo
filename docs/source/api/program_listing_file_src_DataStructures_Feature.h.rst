
.. _program_listing_file_src_DataStructures_Feature.h:

Program Listing for File Feature.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_Feature.h>` (``src/DataStructures/Feature.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <opencv2/core.hpp>
   #include "GlobalDefinitions.h"
   
   namespace  QDVO {
   
   class Feature
   {
   public:
       Feature();
   
       cv::Point_<SCALAR_TYPE> px;
   };
   
   }
