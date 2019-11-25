#include "SlidingWindowEstimator.h"

using namespace QDVO;

SlidingWindowEstimator::SlidingWindowEstimator()
{
    // verify this compiles.
    optimizer.prior.A0.getRowMap<ArgMin::SE3>().begin()->second.getVariableMap<ArgMin::SE3>().begin();
}


void SlidingWindowEstimator::run(QDVO::Graph& graph)
{

}

void SlidingWindowEstimator::removeOutliers(QDVO::Graph& graph)
{

}

void SlidingWindowEstimator::runMarginalizationStrategy(QDVO::Graph& graph)
{

}
