
.. _program_listing_file_src_DataStructures_RadialSearchPattern.h:

Program Listing for File RadialSearchPattern.h
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_DataStructures_RadialSearchPattern.h>` (``src/DataStructures/RadialSearchPattern.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #ifndef RADIALSEARCHPATTERN_H
   #define RADIALSEARCHPATTERN_H
   
   #include <opencv2/core.hpp>
   #include "GlobalDefinitions.h"
   #include <Eigen/Core>
   #include <iostream>
   
   namespace QDVO {
   
   class RadialSearchPattern {
   
   public:
       /*
        * Constructs a radial search pattern which allows for a rapid search for nearest neighbors.
        * Very expensive and meant to be only ran once.
        */
       RadialSearchPattern(const unsigned maxRadius);
   
       RadialSearchPattern(){}
   
       std::vector<std::vector<Eigen::Vector2i> > searchPattern; // index directly correlates to the delta indices which are to be used for a given radius.
   };
   }
   
   #endif // RADIALSEARCHPATTERN_H
