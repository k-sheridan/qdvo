#include "gtest/gtest.h"
#include "SSEOptimizer.h"
#include "Key.h"
#include "Variables/SE3.h"
#include "Variables/InverseDepth.h"
#include <type_traits>

TEST(SSEOptimizer, Basic){
    SE3 pose;
    InverseDepth zinv;

    // Create an optimizer.
 
    //ArgMin::SSEOptimizer<ArgMin::VariableGroup<SE3, InverseDepth>, ArgMin::ErrorTermGroup<SE3, InverseDepth>> optimizer;

    // Add some variables
    //ArgMin::VariableKey<SE3> se3Key = optimizer.addVariable<SE3>(se3);
    //VariableKey<InverseDepth> dinvKey = optimizer.addVariable(InverseDepth());
}