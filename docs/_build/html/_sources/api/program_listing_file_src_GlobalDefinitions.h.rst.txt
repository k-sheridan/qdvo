
.. _program_listing_file_src_GlobalDefinitions.h:

Program Listing for File GlobalDefinitions.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_GlobalDefinitions.h>` (``src/GlobalDefinitions.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #pragma once
   
   #define SCALAR_TYPE double
   #define SMALL_NUMBER 1e-12
   #define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
   
   #include <chrono>
   #include <ctime>
   
   #define TIK auto startT = std::chrono::high_resolution_clock::now();
   #define TOK auto endT = std::chrono::high_resolution_clock::now(); std::chrono::duration<double> diffT = endT-startT; std::cout << "Delta T: " << diffT.count() * 1000 << " ms\n";
   
   #define RETIK startT = std::chrono::high_resolution_clock::now();
   #define RETOK endT = std::chrono::high_resolution_clock::now(); diffT = endT-startT; std::cout << "Delta T: " << diffT.count() * 1000 << " ms\n";
   
   #define ID_TYPE uint64_t
   
   // configurable
   
   // the number of keyframes used in the sliding window estimator
   #define N_KEYFRAMES 7
   
   // the width of the image patch used. must be an odd number.
   #define PATCH_RADIUS 5
   // 2*patch radius + 1
   #define PATCH_WIDTH 11
   
   // which patch comparison will be used.
   #define ZNCC_PATCH_COMPARISON
   
   // Potential correspondence threshold
   #define POTENTIAL_CORRESPONDENCE_THRESHOLD 0.9
   
   // the number of features the feature detector will find on every keyframe.
   #define N_FEATURES_DESIRED 200
   
   // the minimum number of active landmarks allowed before uninitialized landmarks are made active.
   #define MINUMUM_ACTIVE_LANDMARKS 100
   
   // define the maximum radius for the correspondence search.
   #define MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS 15
   
   // the number of extra radii that are searched after finding the first potential correspondence
   #define SEARCH_RADIUS_PADDING 2
   
   // define how many image pyramid levels are used during optiizations
   #define IMAGE_PYRAMID_LEVELS 1
   
   // define the default landmark inverse depth. This is the depth all landmarks are initialized with and for a monocular use case, will influence the scene scale.
   #define DEFAULT_LANDMARK_DINV 0.5
   
   
   
   // not for configuration
   
   // used in the precomputation of the standard deviation / mean lookup table
   // this is used to compute the size of the table which will be upscaled to the full image size.
   #define IMAGE_STDDEV_RESOLUTION (PATCH_RADIUS)*4 + 1
   
   #define EQUIDISTANT_CAMERA_MODEL_RADIUS_MAP_RESOLUTION 0.01
