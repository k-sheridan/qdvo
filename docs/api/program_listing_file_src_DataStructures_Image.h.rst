
.. _program_listing_file_src_DataStructures_Image.h:

Program Listing for File Image.h
================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_Image.h>` (``src/DataStructures/Image.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #include <Eigen/Core>
   #include <opencv2/core.hpp>
   #include "GlobalDefinitions.h"
   #include "Types.h"
   
   namespace QDVO {
   
       typedef float ImageIntensityType;
       typedef Eigen::Matrix<ImageIntensityType, Eigen::Dynamic, Eigen::Dynamic> ImageType;
   
       class Image {
           public:
           Image(cv::Mat& cvImage);
           Image(){}
   
           cv::Mat toOpenCVImage();
   
           ImageType& getImageData(){return image;}
   
           int rows(){return image.rows();}
           int cols(){return image.cols();}
   
           QDVO::Result<ImageIntensityType> getSubPixelIntensity(QDVO::Vector2 px);
   
           private:
           ImageType image;
       };
   }
