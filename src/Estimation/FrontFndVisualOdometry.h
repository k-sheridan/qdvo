#pragma once

#include "Graph.h"
#include "InverseDepth.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

