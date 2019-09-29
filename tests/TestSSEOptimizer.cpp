#include "gtest/gtest.h"
#include "SSEOptimizer.h"
#include "Variables/SE3.h"
#include "Variables/InverseDepth.h"

TEST(SSEOptimizer, Basic){
    SE3 pose;
    InverseDepth zinv;

    //LittleOptimizer::Optimizer<LittleOptimizer::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>, LittleOptimizer::ErrorTermGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>> optimizer;
}