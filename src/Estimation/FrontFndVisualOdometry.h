#pragma once

#include "Graph.h"
#include "SSEOptimizer.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

