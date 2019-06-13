#pragma once

#define SCALAR_TYPE double
#define SMALL_NUMBER 1e-12
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286
#define TIK auto startT = std::chrono::high_resolution_clock::now();
#define TOK auto endT = std::chrono::high_resolution_clock::now(); std::chrono::duration<double> diffT = endT-startT; std::cout << "Delta T: " << diffT.count() * 1000 << " ms\n";
#define ID_TYPE uint64_t
#define EQUIDISTANT_CAMERA_MODEL_RADIUS_MAP_RESOLUTION 0.01


// configurable

// the number of keyframes used in the sliding window estimator
#define N_KEYFRAMES 7

// the width of the image patch used. must be an odd number.
#define PATCH_WIDTH 11

// the number of features the feature detector will find on every keyframe.
#define N_FEATURES_DESIRED 400


// not for configuration

// used in the precomputation of the standard deviation / mean lookup table
// this is used to compute the size of the table which will be upscaled to the full image size.
#define IMAGE_STDDEV_RESOLUTION PATCH_WIDTH
