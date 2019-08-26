#pragma once

#include "Graph.h"
#include "LittleOptimizer.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

