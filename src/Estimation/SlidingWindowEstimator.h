#ifndef SLIDINGWINDOWESTIMATOR_H
#define SLIDINGWINDOWESTIMATOR_H

#include "Graph.h"

class SlidingWindowEstimator
{
public:
    SlidingWindowEstimator();

    void run(QDVO::Graph& graph);

    void removeOutliers(QDVO::Graph& graph);

    void runMarginalizationStrategy(QDVO::Graph& graph);
};

#endif // SLIDINGWINDOWESTIMATOR_H
