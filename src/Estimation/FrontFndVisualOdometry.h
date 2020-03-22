#pragma once

#include "DataStructures/Graph.h"
#include "CameraModel.hpp"
#include "DataStructures/Landmark.h"
#include "DataStructures/Frame.h"
#include "Optimizer/PSDSchurSolver.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/HuberLossFunction.h"
#include "Optimizer/Key.h"
#include "ErrorTerms/QuasiDirectErrorTerm_TargetFrame.h"
#include "Logging.h"

namespace QDVO {

class FrontEndVisualOdometry
{
public:

    /// Variable container which stores the pose refined by the optimizer.
    ArgMin::VariableContainer<ArgMin::SE3> variableContainer;

    /// Stores all error terms used during the optimization.
    ArgMin::ErrorTermContainer<QuasiDirectErrorTerm_TargetFrame> errorTermContainer;

    /// A prior used for the solve.
    ArgMin::GaussianPrior<ArgMin::Scalar<double>, ArgMin::VariableGroup<ArgMin::SE3>> prior;
    
    using LossFunction = ArgMin::HuberLossFunction<double>;

    LossFunction lossFunction = LossFunction(1);

    using Solver = ArgMin::PSDSchurSolver<ArgMin::Scalar<double>, ArgMin::LossFunction<LossFunction>, ArgMin::ErrorTermGroup<QuasiDirectErrorTerm_TargetFrame>, ArgMin::VariableGroup<ArgMin::SE3>, ArgMin::VariableGroup<>>;

    /// Solver used to refine the pose.
    Solver solver = Solver(lossFunction);

    /// The key to the pose being refined.
    ArgMin::VariableKey<ArgMin::SE3> poseKey;

    /**
     * Initializes the solver.
     */
    FrontEndVisualOdometry();

    /**
     * Sets up all error terms for the current frame, and solves for the current frame pose.
     * @param graph The graph containing all keyframes, landmarks, etc.
     */
    void run(QDVO::Graph& graph);
};

} // namespace

