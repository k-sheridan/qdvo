#ifndef FRONTFNDVISUALODOMETRY_H
#define FRONTFNDVISUALODOMETRY_H

#include "Graph.h"
#include "ceres/tiny_solver.h"

class FrontEndVisualOdometry
{
public:
    FrontEndVisualOdometry();

    void run(QDVO::Graph& graph);
};

#endif // FRONTFNDVISUALODOMETRY_H
