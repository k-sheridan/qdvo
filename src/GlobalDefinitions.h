#pragma once

#define SCALAR_TYPE double
#define SMALL_NUMBER 1e-12
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286

#define QDVO_ENABLE_PROFILING

// configurable

// the number of keyframes used in the sliding window estimator
#define N_KEYFRAMES 8

// the width of the image patch used. must be an odd number.
#define PATCH_RADIUS 5
// 2*patch radius + 1
#define PATCH_WIDTH 11

// which patch comparison will be used.
#define ZNCC_PATCH_COMPARISON

// Potential correspondence threshold
#define POTENTIAL_CORRESPONDENCE_THRESHOLD 0.9

// the number of features the feature detector will find on every keyframe.
#define N_FEATURES_DESIRED 300  

// The number of active landmarks desired for a frame.
#define N_ACTIVE_LANDMARKS_DESIRED 200

// The minimum active landmark seperation
#define MINUMUM_LANDMARK_SEPERATION 5

// the minimum number of active landmarks allowed before uninitialized landmarks are made active.
#define MINUMUM_ACTIVE_LANDMARKS 100

// define the maximum radius for the correspondence search.
#define MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS 25

// the number of extra radii that are searched after finding the first potential correspondence
#define SEARCH_RADIUS_PADDING 5

// define how many image pyramid levels are used during optiizations
#define IMAGE_PYRAMID_LEVELS 4

// define the default landmark inverse depth. This is the depth all landmarks are initialized with and for a monocular use case, will influence the scene scale.
#define DEFAULT_LANDMARK_DINV 0.4



// not for configuration

// used in the precomputation of the standard deviation / mean lookup table
// this is used to compute the size of the table which will be upscaled to the full image size.
#define IMAGE_STDDEV_RESOLUTION (PATCH_RADIUS)*4 + 1

#define EQUIDISTANT_CAMERA_MODEL_RADIUS_MAP_RESOLUTION 0.01
