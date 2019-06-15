#ifndef BASICALGORITHM_H
#define BASICALGORITHM_H

#include "GlobalDefinitions.h"
#include "Graph.h"
#include "FrontFndVisualOdometry.h"
#include "SlidingWindowEstimator.h"
#include "ImageStatisticsLUT.h"
#include "PatchComparer.h"
#include "RadialSearchPattern.h"
#include <opencv2/core.hpp>

namespace QDVO {

/*
 * The goal of this class is to be a "easily" modifiable implementation of the quasi-direct approach to visual odometry.
 * This should allow easy modification of the patch comparison, and serve as a place for preallocation of quantities.
 *
 * The basic datastructures are held in the graph datastructure. They contain all necessary information, access, insert, and reset functions implemented in a protected way.
 *
 */
class BasicAlgorithm
{
public:
    BasicAlgorithm();

    Graph graph; // main datastructure for all possible QDVO algorithms.

    FrontEndVisualOdometry frontEndVisualOdometry; // used to initialize the current frame pose.

    SlidingWindowEstimator swe; // main estimator for the whole algorithm

    /*
     * Preallocates keyframes and current frame.
     */
    void initialize();

    /*
     * Gives the algorithm a new image
     */
    void addImage(cv::Mat& image, const double& time, const ID_TYPE cameraID = 1);

    /*
     * adds a new camera for the visual odometry algorithm.
     */
    void addCamera(std::unique_ptr<QDVO::CameraModel>& cameraModel, const ID_TYPE cameraID = 1);

    /*
     * detects features and adds new landmarks to the frame.
     */
    void createNewLandmarks(QDVO::Frame& keyframe);

    /*
     * Activates new landmarks from the set of keyframes such that the current features are well distributed.
     */
    void activateNewLandmarks();

    /*
     * Does an initial search for potential correspondences between active landmarks and the current frame.
     */
    void initializeCurrentFrameCorrespondenceDistributions();

    /*
     * Checks if the current frame has met the criteria for a keyframe.
     */
    bool isCurrentFrameAKeyframe();



    // -=-=-=-===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // ZNCC IMPLEMENTATION

    std::unordered_map<ID_TYPE, std::unique_ptr<PatchComparer> > patchComparers; // used to speed up the patch comparisons.

    std::shared_ptr<RadialSearchPattern> radialSearchPatternPtr; // used globally to generate the correspondence distributions.
};

}

#endif // BASICALGORITHM_H
