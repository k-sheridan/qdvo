
.. _program_listing_file_src_DataStructures_Image.cpp:

Program Listing for File Image.cpp
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_Image.cpp>` (``src/DataStructures/Image.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include "Image.h"
   #include <opencv2/core/eigen.hpp>
   
   
   QDVO::Result<QDVO::ImageIntensityType> QDVO::Image::getSubPixelIntensity(QDVO::Vector2 px)
   {
               int x0 = (int)px.x();
               int y0 = (int)px.y();
               int x1 = x0 + 1;
               int y1 = y0 + 1;
   
               if ((x0 >= (this->cols() - 1) || x0 < 0) || (y0 >= (this->rows() - 1) || y0 < 0))
               {
                   //throw std::runtime_error("pixel out of bounds");
                   return {};
               }
   
               float a = px.x() - (float)x0;
               float c = px.y() - (float)y0;
   
               return ((image(y0, x0) * (1.f - a) + image(y0, x1) * a) * (1.f - c)
                                      + (image(y1, x0) * (1.f - a) + image(y1, x1) * a) * c);
   }
