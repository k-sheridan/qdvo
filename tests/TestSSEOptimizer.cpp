#include "gtest/gtest.h"
#include "SSEOptimizer.h"
#include "Variables/SE3.h"
#include "Variables/InverseDepth.h"

TEST(SSEOptimizer, Basic){
    SE3 pose;
    InverseDepth zinv;

    ArgMin::SSEOptimizer<ArgMin::VariableGroup<SE3, InverseDepth>, ArgMin::ErrorTermGroup<SE3, InverseDepth>> optimizer;
}