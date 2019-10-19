#include "gtest/gtest.h"
#include "Optimizer/SSEOptimizer.h"
#include "Optimizer/Key.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/Variables/InverseDepth.h"
#include <type_traits>

TEST(ArgMin, Basic)
{
    SE3 pose;
    InverseDepth zinv;

    // Create an optimizer.

    ArgMin::SSEOptimizer<double, ArgMin::VariableGroup<SE3, InverseDepth>, ArgMin::ErrorTermGroup<SE3, InverseDepth>> optimizer;

    // Add some variables
    //ArgMin::VariableKey<SE3> se3Key = optimizer.addVariable<SE3>(se3);
    //VariableKey<InverseDepth> dinvKey = optimizer.addVariable(InverseDepth());
}

TEST(ArgMin, SparseBlockRow)
{
}