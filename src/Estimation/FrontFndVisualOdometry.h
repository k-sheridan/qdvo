#pragma once

#include "Graph.h"
#include "Optimizer.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

