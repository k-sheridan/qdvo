#pragma once

#include "DataStructures/Graph.h"
#include "CameraModel.hpp"
#include "DataStructures/Landmark.h"
#include "DataStructures/Frame.h"
#include "Optimizer/SSEOptimizer.h"

namespace QDVO {

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

} // namespace

