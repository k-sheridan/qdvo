#pragma once

#include "DataStructures/Graph.h"
#include "DataStructures/Landmark.h"
#include "DataStructures/Frame.h"
#include "CameraModel.hpp"
#include "Optimizer/SSEOptimizer.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/Key.h"
#include "Optimizer/Containers.h"

namespace QDVO {

class SlidingWindowEstimator
{
public:

    SlidingWindowEstimator();

    void run(QDVO::Graph& graph);

    void removeOutliers(QDVO::Graph& graph);

    void runMarginalizationStrategy(QDVO::Graph& graph);

};

} // namespace QDVO
