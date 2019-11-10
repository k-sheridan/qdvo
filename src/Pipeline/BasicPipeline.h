#pragma once

#include "GlobalDefinitions.h"
#include "DataStructures/Graph.h"
#include "FrontFndVisualOdometry.h"
#include "SlidingWindowEstimator.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "RadialSearchPattern.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Types.h"
#include "Optimizer/ParallelAlgorithms/ParallelAlgorithms.h"

namespace QDVO
{

/*
 * The goal of this class is to be a "easily" modifiable implementation of the quasi-direct approach to visual odometry.
 * This should allow easy modification of the patch comparison, and serve as a place for preallocation of quantities.
 *
 * The basic datastructures are held in the graph datastructure. They contain all necessary information, access, insert, and reset functions implemented in a protected way.
 *
 */
class BasicPipeline
{
public:
    BasicPipeline();

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
    void addFrame(cv::Mat &image, const double &time, const ID_TYPE cameraID = 1);

    /*
     * adds a new camera for the visual odometry algorithm.
     */
    void addCamera(std::unique_ptr<QDVO::CameraModel> &cameraModel, const ID_TYPE cameraID = 1);

    /*
     * detects features and adds new landmarks to the frame.
     */
    void createNewLandmarks(std::unique_ptr<QDVO::Frame> &keyframe, std::unique_ptr<QDVO::FeatureDetector> &featureDetector);

    /*
     * Activates new landmarks from the set of keyframes such that the current features are well distributed.
     */
    void activateNewLandmarks();

    /*
     * Does an initial search for potential correspondences between active landmarks and the current frame.
     */
    void initializeCorrespondenceDistributionsForCurrentFrame();

    /*
     * Attempts to initialize inactive landmarks using an epipolar search
     */
    void runEpipolarDepthEstimators();

    /*
     * marginalizes keyframes. This function can be changed in a modular fashion
     */
    void runMarginalizationStrategy();

    /*
     * Checks if the current frame has met the criteria for a keyframe.
     */
    bool isCurrentFrameAKeyframe();

    /*
     * This function is ran after the current frame is setup and before the correspondence distributions are initialized. It will insert a new patch comparer
     * into the unordered_map of patch comparers while removing any old/redundant patch comparers.
     */
    void updatePatchComparers();

    // -=-=-=-===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // ZNCC IMPLEMENTATION SPECIFIC and PREALLOCATED / PRECOMPUTED DATA

    std::unique_ptr<QDVO::FeatureDetector> featureDetector; // global feature detector. NOT MEANT TO BE USED IN MULTIPLE THREADS WITHOUT LOCKING!

    std::unordered_map<ID_TYPE, std::shared_ptr<PatchComparer>> patchComparers; // used to speed up the patch comparisons.

    std::shared_ptr<RadialSearchPattern> radialSearchPatternPtr; // used globally to generate the correspondence distributions.

    std::unique_ptr<QDVO::PatchWarper> patchWarper; // pointer to the patch warp implementation
};

} // namespace QDVO

