#include "gtest/gtest.h"
#include "LittleOptimizer.h"
#include "SparseBlockMatrix.h"
#include "SE3.h"
#include "InverseDepth.h"

TEST(SparseBlockMatrix, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    LittleOptimizer::SparseBlockMatrix::Scalar<double> ScalarType;
    LittleOptimizer::SparseBlockMatrix::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth> VariableTypes;

    LittleOptimizer::SparseBlockMatrix::SparseBlockMatrix< LittleOptimizer::SparseBlockMatrix::Scalar<double>, LittleOptimizer::SparseBlockMatrix::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth> > matrix;

}

TEST(LittleOptimizer, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    LittleOptimizer::Optimizer<LittleOptimizer::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>, LittleOptimizer::ErrorTermGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>> optimizer;
}