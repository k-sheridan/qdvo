#pragma once

#include "DataStructures/Graph.h"
#include "Optimizer/SSEOptimizer.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

