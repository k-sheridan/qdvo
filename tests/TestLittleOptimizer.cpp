#include "gtest/gtest.h"
#include "LittleOptimizer.h"
#include "SparseBlockMatrix.h"
#include "SE3.h"
#include "InverseDepth.h"

TEST(SparseBlockMatrix, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    using ScalarType = LittleOptimizer::SparseBlockMatrix::Scalar<double>;
    using VariableTypes = LittleOptimizer::SparseBlockMatrix::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>;

    LittleOptimizer::SparseBlockMatrix::SparseBlockMatrix< ScalarType, VariableTypes > matrix;

    using SE3Key = LittleOptimizer::VariableKey<LittleOptimizer::SE3>;
    using DinvKey = LittleOptimizer::VariableKey<LittleOptimizer::InverseDepth>;

    SE3Key k1;
    k1.index = 0;
    DinvKey k2;
    k2.index = 0;

    Eigen::Matrix<double, 6, 1> subMatrix = matrix.get(k1, k2);

}

TEST(LittleOptimizer, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    LittleOptimizer::Optimizer<LittleOptimizer::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>, LittleOptimizer::ErrorTermGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>> optimizer;
}