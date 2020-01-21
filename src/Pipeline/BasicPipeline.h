#pragma once

#include "GlobalDefinitions.h"
#include "DataStructures/Graph.h"
#include "FrontFndVisualOdometry.h"
#include "SlidingWindowEstimator.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "DataStructures/RadialSearchPattern.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "Types.h"
#include "Optimizer/ParallelAlgorithms/ParallelAlgorithms.h"

#include "spdlog/spdlog.h"

namespace QDVO
{

/**
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

    /// main datastructure for all possible QDVO algorithms.
    Graph graph;

    /// used to initialize the current frame pose.
    FrontEndVisualOdometry frontEndVisualOdometry;

    /// main estimator for the whole algorithm.
    SlidingWindowEstimator swe; 

    /**
     * Preallocates keyframes and current frame.
     */
    void initialize();

    /**
     * Gives the algorithm a new image
     */
    void addFrame(cv::Mat &image, const double &time, const CameraModelMap::key_type& cameraModelKey, const ExtrinsicMap::key_type& extrinsicKey);

    /**
     * adds a new camera for the visual odometry algorithm.
     */
    CameraModelMap::key_type addCamera(std::unique_ptr<QDVO::CameraModel> &cameraModel);

    /**
     * detects features and adds new landmarks to the frame.
     */
    void createNewLandmarks(Graph& graph, KeyframeMap::key_type keyframeKey, std::unique_ptr<QDVO::FeatureDetector> &featureDetector);

    /**
     * Activates new landmarks from the set of keyframes such that the current features are well distributed.
     */
    void activateNewLandmarks();

    /**
     * Does an initial search for potential correspondences between active landmarks and the current frame.
     */
    void initializeCorrespondenceDistributionsForCurrentFrame();

    /**
     * Attempts to initialize inactive landmarks using an epipolar search
     */
    void runEpipolarDepthEstimators(KeyframeMap::key_type mostRecentKeyframeKey);

    /**
     * marginalizes keyframes. This function can be changed in a modular fashion
     */
    void runMarginalizationStrategy();

    /**
     * Checks if the current frame has met the criteria for a keyframe.
     */
    bool isCurrentFrameAKeyframe();

    /**
     * Computes the pixel flow and translational pixel flow between the current frame and a desired keyframe.
     * If a pixel flow is nan, there were no commonly observed features between the two frames.
     * @param keyframeKey the keyframe used to compute pixel flow with respect to.
     * @return A pair representing the pixel flow and translational pixel flow in that order.
     */
    std::pair<double, double> computePixelFlowForCurrentFrame(KeyframeMap::key_type keyframeKey);

    /**
     * This function is ran after the current frame is setup and before the correspondence distributions are initialized. It will insert a new patch comparer
     * into the unordered_map of patch comparers while removing any old/redundant patch comparers.
     */
    void updatePatchComparers();

    // -=-=-=-===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // ZNCC IMPLEMENTATION SPECIFIC and PREALLOCATED / PRECOMPUTED DATA

    /// global feature detector. NOT MEANT TO BE USED IN MULTIPLE THREADS WITHOUT LOCKING!
    std::unique_ptr<QDVO::FeatureDetector> featureDetector;

    /// used to speed up the patch comparisons.
    std::shared_ptr<PatchComparer> patchComparer;

    /// used globally to generate the correspondence distributions.
    std::shared_ptr<RadialSearchPattern> radialSearchPatternPtr;

    /// pointer to the patch warp implementation
    std::unique_ptr<QDVO::PatchWarper> patchWarper;
};

} // namespace QDVO

