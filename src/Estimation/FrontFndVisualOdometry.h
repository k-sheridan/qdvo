#pragma once

#include "DataStructures/Graph.h"
#include "Optimizer/SSEOptimizer.h"

namespace QDVO {

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

} // namespace

