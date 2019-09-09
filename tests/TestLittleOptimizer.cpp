#include "gtest/gtest.h"
#include "LittleOptimizer.h"
#include "SparseBlockMatrix.h"
#include "SparseBlockVector.h"
#include "SE3.h"
#include "InverseDepth.h"

TEST(SparseBlockMatrix, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    using ScalarType = LittleOptimizer::Scalar<double>;
    using VariableTypes = LittleOptimizer::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>;

    LittleOptimizer::SparseBlockMatrix::SparseBlockMatrix< ScalarType, VariableTypes > matrix;
    LittleOptimizer::SparseBlockMatrix::SparseBlockMatrix< ScalarType, VariableTypes > matrix2;

    using SE3Key = LittleOptimizer::TypedIndex<LittleOptimizer::SE3>;
    using DinvKey = LittleOptimizer::TypedIndex<LittleOptimizer::InverseDepth>;

    SE3Key k1;
    k1.index = 0;
    DinvKey k2;
    k2.index = 0;

    matrix += matrix2;

    matrix.addRow<LittleOptimizer::SE3>();
    matrix.addRow<LittleOptimizer::InverseDepth>();

    std::cout << matrix.rowsOfType<LittleOptimizer::SE3>() << ", " << matrix.columnsOfType<LittleOptimizer::SE3>() << std::endl;

    LittleOptimizer::SubMatrix<double, 6, 1> subMatrix = matrix.get(k1, k2);
    LittleOptimizer::SubMatrix<double, 6, 6> subMatrix2 = matrix.get(k1, k1);

    matrix.clear();

}

TEST(SparseBlockVector, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    using ScalarType = LittleOptimizer::Scalar<double>;
    using VariableTypes = LittleOptimizer::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>;

    LittleOptimizer::SparseBlockVector::SparseBlockVector< ScalarType, VariableTypes > matrix;
    LittleOptimizer::SparseBlockVector::SparseBlockVector< ScalarType, VariableTypes > matrix2;

    using SE3Key = LittleOptimizer::TypedIndex<LittleOptimizer::SE3>;
    using DinvKey = LittleOptimizer::TypedIndex<LittleOptimizer::InverseDepth>;

    SE3Key k1;
    k1.index = 0;
    DinvKey k2;
    k2.index = 0;

    matrix.addRow<LittleOptimizer::SE3>();
    matrix.addRow<LittleOptimizer::InverseDepth>();

    matrix2.addRow<LittleOptimizer::SE3>();
    matrix2.addRow<LittleOptimizer::InverseDepth>();

    matrix += matrix2;

    LittleOptimizer::SubMatrix<double, 6, 1> subMatrix = matrix.get(k1);
    LittleOptimizer::SubMatrix<double, 1, 1> subMatrix2 = matrix.get(k2);

}

TEST(LittleOptimizer, Basic){
    LittleOptimizer::SE3 pose;
    LittleOptimizer::InverseDepth zinv;

    LittleOptimizer::Optimizer<LittleOptimizer::VariableGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>, LittleOptimizer::ErrorTermGroup<LittleOptimizer::SE3, LittleOptimizer::InverseDepth>> optimizer;
}