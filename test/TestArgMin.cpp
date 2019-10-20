#include "gtest/gtest.h"
#include "Optimizer/SSEOptimizer.h"
#include "Optimizer/SparseBlockRow.h"
#include "Optimizer/SparseBlockMatrix.h"
#include "Optimizer/Key.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/Variables/InverseDepth.h"
#include <type_traits>

using namespace ArgMin;

TEST(ArgMin, Basic)
{
    SE3 pose;
    InverseDepth zinv;

    // Create an optimizer.

    ArgMin::SSEOptimizer<Scalar<double>, ArgMin::VariableGroup<SE3, InverseDepth>, ArgMin::ErrorTermGroup<SE3, InverseDepth>> optimizer;

    // Add some variables
    //ArgMin::VariableKey<SE3> se3Key = optimizer.addVariable<SE3>(se3);
    //VariableKey<InverseDepth> dinvKey = optimizer.addVariable(InverseDepth());
}

TEST(ArgMin, SparseBlockRowOperations)
{
    SE3 pose;
    InverseDepth zinv;

    using SBR = ArgMin::SparseBlockRow<Scalar<double>, Dimension<2>, ArgMin::VariableGroup<SE3, InverseDepth>>;

    SBR sbr;

    VariableContainer<SE3, InverseDepth> variableContainer;

    // insert variables
    auto se3Key1 = variableContainer.getVariableMap<SE3>().insert(pose);
    auto se3Key2 = variableContainer.getVariableMap<SE3>().insert(pose);

    auto dinvKey1 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);
    auto dinvKey2 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);

    SBR::MatrixBlock<SE3> matrix = SBR::MatrixBlock<SE3>::Constant(1);
    sbr.getVariableMap<SE3>().insert(std::make_pair(se3Key1, matrix));
    matrix = SBR::MatrixBlock<SE3>::Constant(2);
    sbr.getVariableMap<SE3>().insert(std::make_pair(se3Key2, matrix));

    SBR::MatrixBlock<InverseDepth> matrix2 = SBR::MatrixBlock<InverseDepth>::Constant(3);
    sbr.getVariableMap<InverseDepth>().insert(std::make_pair(dinvKey1, matrix2));

    // prepare a vector for multiplication
    EXPECT_EQ(variableContainer.totalDimensions(), 14);

    Eigen::Matrix<double, Eigen::Dynamic, 1> dx;
    dx.resize(variableContainer.totalDimensions(), 1);

    EXPECT_EQ(variableContainer.variableIndex(se3Key1), 0);
    EXPECT_EQ(variableContainer.variableIndex(se3Key2), 6);
    EXPECT_EQ(variableContainer.variableIndex(dinvKey1), 12);
    EXPECT_EQ(variableContainer.variableIndex(dinvKey2), 13);

    // try dotting with zeros
    dx.setZero();
    Eigen::Matrix<double, 2, 1> result;
    sbr.dot(variableContainer, dx, result);
    EXPECT_EQ(result(0, 0), 0);
    EXPECT_EQ(result(1, 0), 0);

    // set blocks to zero.
    sbr.setZero();

    EXPECT_EQ(sbr.getVariableMap<SE3>().at(se3Key1)(0, 0), 0);
    EXPECT_EQ(sbr.getVariableMap<SE3>().at(se3Key2)(0, 0), 0);
    EXPECT_EQ(sbr.getVariableMap<InverseDepth>().at(dinvKey1)(0, 0), 0);
}

TEST(ArgMin, SparseBlockMatrixOperations)
{
    SE3 pose;
    InverseDepth zinv;

    using SBM = ArgMin::SparseBlockMatrix<Scalar<double>, ArgMin::VariableGroup<SE3, InverseDepth>>;

    SBM sbm;

    VariableContainer<SE3, InverseDepth> variableContainer;

    // insert variables
    auto se3Key1 = variableContainer.getVariableMap<SE3>().insert(pose);
    auto se3Key2 = variableContainer.getVariableMap<SE3>().insert(pose);

    auto dinvKey1 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);
    auto dinvKey2 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);

    // Test manual insert. This obviously needs to be simplified.
    sbm.getRowMap<SE3>().insert(std::make_pair(se3Key1, SBM::Row<SE3>()));
    Eigen::Matrix<double, 6, 6> matrix = Eigen::Matrix<double, 6, 6>::Identity();
    sbm.getRowMap<SE3>().at(se3Key1).getVariableMap<SE3>().insert(std::make_pair(se3Key1, matrix));

    Eigen::Matrix<double, Eigen::Dynamic, 2> v = Eigen::Matrix<double, 14, 2>::Ones();

    // Test sparse dot product.
    Eigen::Matrix<double, Eigen::Dynamic, 2> result;
    result.resize(14, 2);
    sbm.dot(variableContainer, v, result);

    Eigen::Matrix<double, Eigen::Dynamic, 2> expectedResult = Eigen::Matrix<double, 14, 2>::Zero();
    expectedResult.block(0,0, 6,2) = Eigen::Matrix<double, 6, 2>::Ones();

    EXPECT_EQ(result, expectedResult);


}