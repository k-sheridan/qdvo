#pragma once

#include "DataStructures/Graph.h"

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
