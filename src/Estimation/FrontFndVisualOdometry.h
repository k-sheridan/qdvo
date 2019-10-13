#pragma once

#include "DataStructures/Graph.h"
#include "SSEOptimizer.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

