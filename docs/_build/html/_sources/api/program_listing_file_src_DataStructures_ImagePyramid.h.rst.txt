
.. _program_listing_file_src_DataStructures_ImagePyramid.h:

Program Listing for File ImagePyramid.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_ImagePyramid.h>` (``src/DataStructures/ImagePyramid.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include "GlobalDefinitions.h"
   #include "Image.h"
   
   namespace QDVO {
   class ImagePyramid
   {
   public:
       ImagePyramid(int levels = 1);
   
       QDVO::Image& getImage(const size_t level = 0);
   
       void generate(cv::Mat& baseImage);
   
       size_t levels() const {return this->imageLevels.size();}
   
   protected:
       std::vector<QDVO::Image> imageLevels; // level 0 is full resolution.
   
   };
   }
   
