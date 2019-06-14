#ifndef BASICALGORITHM_H
#define BASICALGORITHM_H

#include <GlobalDefinitions.h>
#include <Graph.h>
#include <Estimation/FrontFndVisualOdometry.h>
#include <Estimation/SlidingWindowEstimator.h>
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
    ID_TYPE addCamera(std::unique_ptr<QDVO::CameraModel>& cameraModel);

    /*
     * detects features and adds new landmarks to the frame.
     */
    void createNewLandmarks(QDVO::Frame& keyframe);



    // -=-=-=-===-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // ZNCC IMPLEMENTATION

};

}

#endif // BASICALGORITHM_H
