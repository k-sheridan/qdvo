#pragma once

#include "Types.h"
#include "DataStructures/Graph.h"
#include "DataStructures/Landmark.h"
#include "DataStructures/Frame.h"
#include "CameraModel.hpp"
#include "Optimizer/SSEOptimizer.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/Variables/InverseDepth.h"
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/GaussianPrior.h"
#include "Optimizer/HuberLossFunction.h"
#include "Optimizer/PSDSchurSolver.h"
#include "Optimizer/Key.h"
#include "Optimizer/Containers.h"
#include "ErrorTerms/QuasiDirectErrorTerm.h"
#include "Optimizer/SlotArray.h"
#include "Optimizer/SlotMap.h"
#include "Optimizer/Key.h"

namespace QDVO {

/**
 * The Sliding Window Estimator attempts the estimate the states of N camera frames and the depths of the landmarks
 * which the observe.
 * 
 * This estimator marginalizes camera frames and landmarks hosted in them into a single gaussian prior when the frame is
 * deemed not useful.
 * 
 * Landmarks will be culled if they are deemed to be outliers.
 */
class SlidingWindowEstimator
{
public:

    /// Variable container which stores the pose refined by the optimizer.
    ArgMin::VariableContainer<ArgMin::SE3, ArgMin::InverseDepth> variableContainer;

    /// Stores all error terms used during the optimization.
    ArgMin::ErrorTermContainer<QDVO::QuasiDirectErrorTerm> errorTermContainer;

    /// A prior used for the solve.
    ArgMin::GaussianPrior<ArgMin::Scalar<double>, ArgMin::VariableGroup<ArgMin::SE3, ArgMin::InverseDepth>> prior;
    
    using LossFunction = ArgMin::HuberLossFunction<double>;

    LossFunction lossFunction = LossFunction(1000);

    using Solver = ArgMin::PSDSchurSolver<ArgMin::Scalar<double>, ArgMin::LossFunction<LossFunction>, ArgMin::ErrorTermGroup<QDVO::QuasiDirectErrorTerm>, ArgMin::VariableGroup<ArgMin::SE3, ArgMin::InverseDepth>, ArgMin::VariableGroup<ArgMin::InverseDepth>>;

    /// Solver used to refine the pose.
    Solver solver = Solver(lossFunction);
    
    /// A map between a variable key used by the solver and a key used in the graph for camera poses.
    ArgMin::SlotArray<ArgMin::VariableKey<ArgMin::SE3>, KeyframeMap::key_type> poseKeyMap;

    /// A map between the variable key used in the solver and a landmark key for the inverse depth. 
    ArgMin::SlotArray<ArgMin::VariableKey<ArgMin::InverseDepth>, LandmarkMap::key_type> dinvKeyMap;

    SlidingWindowEstimator();

    /**
     * Checks to see if a new keyframe has been added to the graph, inserts its variables and error terms
     * into the slover, and solves for an the minimum error.
     * 
     * @param graph The graph containing landmarks, keyframes, and observations.
     */
    void run(QDVO::Graph& graph);

    /**
     * Checks if the current solver contains any outliers, and culls them from both the solver, and graph.
     * 
     * @param graph The graph containing landmarks, keyframes, and observations.
     */
    void removeOutliers(QDVO::Graph& graph);

    /**
     * Looks through the graph for a keyframe which provides the least information according to a 
     * distance heuristic. The keyframe's information is then approximated into the gaussian prior and removed from both the
     * graph, solver, and prior along with any error terms containing references to it.
     * 
     * @param graph The graph containing landmarks, keyframes, and observations.
     */
    void runMarginalizationStrategy(QDVO::Graph& graph);

};

} // namespace QDVO
