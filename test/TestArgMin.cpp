#include "gtest/gtest.h"
#include "Optimizer/SSEOptimizer.h"
#include "Optimizer/SparseBlockRow.h"
#include "Optimizer/SparseBlockMatrix.h"
#include "Optimizer/GaussianPrior.h"
#include "Optimizer/BlockVector.h"
#include "Optimizer/PSDSchurSolver.h"
#include "Optimizer/Key.h"
#include "Optimizer/Variables/SE3.h"
#include "Optimizer/Variables/InverseDepth.h"
#include "Optimizer/Variables/SimpleScalar.h"
#include "Optimizer/ErrorTermBase.h"
#include <type_traits>

using namespace ArgMin;

class DifferentSimpleScalar : public SimpleScalar
{
public:
    DifferentSimpleScalar(double val) : SimpleScalar(val)
    {
    }
};

class DifferenceErrorTerm : public ErrorTermBase<Scalar<double>, Dimension<1>, VariableGroup<SimpleScalar, DifferentSimpleScalar>>
{
public:
    DifferenceErrorTerm(VariableKey<SimpleScalar> key1, VariableKey<DifferentSimpleScalar> key2)
    {
        std::get<0>(variableKeys) = key1;
        std::get<1>(variableKeys) = key2;
    }

    template <typename... Variables>
    void evaluate(VariableContainer<Variables...> &variables, bool relinearize)
    {
        EXPECT_TRUE(checkVariablePointerConsistency(variables));

        auto &var1 = *(std::get<0>(variablePointers));
        auto &var2 = *(std::get<1>(variablePointers));

        residual(0, 0) = var2.value - var1.value;

        if (relinearize)
        {
            auto &jac1 = (std::get<0>(variableJacobians));
            auto &jac2 = (std::get<1>(variableJacobians));

            jac1(0, 0) = -1;
            jac2(0, 0) = 1;

            linearizationValid = true;
        }
        else
        {
            linearizationValid = false;
        }
    }
};

TEST(ArgMin, Basic)
{
    SE3 pose;
    InverseDepth zinv;

    // Create an optimizer.

    ArgMin::SSEOptimizer<Scalar<double>, ArgMin::VariableGroup<SE3, InverseDepth>, ArgMin::ErrorTermGroup<DifferenceErrorTerm>> optimizer;

    // Add some variables
    ArgMin::VariableKey<SE3> se3Key = optimizer.addVariable(pose);
    VariableKey<InverseDepth> dinvKey = optimizer.addVariable(zinv);
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

    // Test insert.
    Eigen::Matrix<double, 6, 6> matrix = Eigen::Matrix<double, 6, 6>::Identity();
    sbm.setBlock(se3Key1, se3Key1, matrix);

    Eigen::Matrix<double, Eigen::Dynamic, 2> v = Eigen::Matrix<double, 14, 2>::Ones();

    // Test sparse dot product.
    Eigen::Matrix<double, Eigen::Dynamic, 2> result;
    result.resize(14, 2);
    sbm.dot(variableContainer, v, result);

    Eigen::Matrix<double, Eigen::Dynamic, 2> expectedResult = Eigen::Matrix<double, 14, 2>::Zero();
    expectedResult.block(0, 0, 6, 2) = Eigen::Matrix<double, 6, 2>::Ones();

    EXPECT_EQ(result, expectedResult);

    // insert another diagonal member
    matrix *= 2;
    sbm.setBlock(se3Key2, se3Key2, matrix);
    // insert a inverse depth diagonal.
    auto matrix2 = Eigen::Matrix<double, 1, 1>::Ones();
    sbm.setBlock(dinvKey2, dinvKey2, matrix2);

    expectedResult.block(6, 0, 6, 2) = Eigen::Matrix<double, 6, 2>::Constant(2);
    expectedResult.block(13, 0, 1, 2) = Eigen::Matrix<double, 1, 2>::Ones();

    sbm.dot(variableContainer, v, result);

    EXPECT_EQ(result, expectedResult);

    // insert an off diagonal element
    matrix = Eigen::Matrix<double, 6, 6>::Identity();
    sbm.setBlock(se3Key1, se3Key2, matrix);

    expectedResult.block(0, 0, 6, 2) = Eigen::Matrix<double, 6, 2>::Constant(2);

    sbm.dot(variableContainer, v, result);

    EXPECT_EQ(result, expectedResult);

    // erase an element
    sbm.removeBlock(se3Key1, se3Key1);

    expectedResult.block(0, 0, 6, 2) = Eigen::Matrix<double, 6, 2>::Constant(1);

    sbm.dot(variableContainer, v, result);

    EXPECT_EQ(result, expectedResult);
}

TEST(ArgMin, BlockVector)
{
    SE3 pose;
    InverseDepth zinv;

    using V = ArgMin::BlockVector<Scalar<double>, Dimension<1>, ArgMin::VariableGroup<SE3, InverseDepth>>;

    V vec;

    VariableContainer<SE3, InverseDepth> variableContainer;

    // insert variables
    auto se3Key1 = variableContainer.getVariableMap<SE3>().insert(pose);
    auto se3Key2 = variableContainer.getVariableMap<SE3>().insert(pose);

    auto dinvKey1 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);
    auto dinvKey2 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);

    V::MatrixBlock<SE3> mat = V::MatrixBlock<SE3>::Ones();
    vec.addRowBlock(se3Key1, mat);
    V::MatrixBlock<SE3> mat2 = V::MatrixBlock<SE3>::Constant(2);
    vec.addRowBlock(se3Key2, mat2);

    EXPECT_EQ(vec.getRowBlock(se3Key2), mat2);
    EXPECT_EQ(vec.getRowBlock(se3Key1), mat);

    vec.getRowBlock(se3Key1) = mat2;

    EXPECT_EQ(vec.getRowBlock(se3Key1), mat2);

    vec.getRowBlock(se3Key1) = mat;

    EXPECT_TRUE(vec.blockExists(se3Key1));

    Eigen::Matrix<double, Eigen::Dynamic, 1> dx;
    dx.setOnes(14, 1);

    // subtract the vector from the block vec.
    vec.subtractVector(variableContainer, dx);

    // verify the values
    EXPECT_TRUE(vec.blockExists(se3Key1));
    EXPECT_EQ(vec.getRowBlock(se3Key1), V::MatrixBlock<SE3>::Zero());

    EXPECT_TRUE(vec.blockExists(se3Key2));
    EXPECT_EQ(vec.getRowBlock(se3Key2), V::MatrixBlock<SE3>::Ones());

    EXPECT_TRUE(vec.blockExists(dinvKey1));
    EXPECT_EQ(vec.getRowBlock(dinvKey1), -V::MatrixBlock<InverseDepth>::Ones());

    EXPECT_TRUE(vec.blockExists(dinvKey2));
    EXPECT_EQ(vec.getRowBlock(dinvKey2), -V::MatrixBlock<InverseDepth>::Ones());

    vec.removeRowBlock(se3Key1);

    EXPECT_FALSE(vec.blockExists(se3Key1));

    ASSERT_DEATH(vec.getRowBlock(se3Key1), "");
}

TEST(ArgMin, GaussianPrior)
{
    SE3 pose;
    InverseDepth zinv;

    using Prior = ArgMin::GaussianPrior<Scalar<double>, ArgMin::VariableGroup<SE3, InverseDepth>>;
    Prior prior;

    VariableContainer<SE3, InverseDepth> variableContainer;

    // insert variables
    auto se3Key1 = variableContainer.getVariableMap<SE3>().insert(pose);
    auto se3Key2 = variableContainer.getVariableMap<SE3>().insert(pose);

    auto dinvKey1 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);
    auto dinvKey2 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);

    // Add variables to prior.
    prior.addVariable(se3Key1);
    prior.addVariable(se3Key2);
    prior.addVariable(dinvKey1);
    prior.addVariable(dinvKey2);

    Eigen::Matrix<double, Eigen::Dynamic, 1> dx;
    dx.resize(14, Eigen::NoChange);
    dx.setOnes();

    prior.update(variableContainer, dx);

    EXPECT_TRUE(prior.b0.blockExists(se3Key1));
    std::cout << prior.A0.getRowMap<SE3>().at(se3Key1).getVariableMap<SE3>().at(se3Key1) << std::endl;
    std::cout << prior.b0.getRowBlock(se3Key1) << std::endl;
    EXPECT_TRUE(prior.b0.getRowBlock(se3Key1).isApprox(Prior::BV::MatrixBlock<SE3>::Constant(-Prior::DefaultInverseVariance)));

    EXPECT_TRUE(prior.b0.blockExists(se3Key2));
    EXPECT_TRUE(prior.b0.getRowBlock(se3Key2).isApprox(Prior::BV::MatrixBlock<SE3>::Constant(-Prior::DefaultInverseVariance)));

    EXPECT_TRUE(prior.b0.blockExists(dinvKey1));
    EXPECT_TRUE(prior.b0.getRowBlock(dinvKey1).isApprox(Prior::BV::MatrixBlock<InverseDepth>::Constant(-Prior::DefaultInverseVariance)));

    EXPECT_TRUE(prior.b0.blockExists(dinvKey2));
    EXPECT_TRUE(prior.b0.getRowBlock(dinvKey2).isApprox(Prior::BV::MatrixBlock<InverseDepth>::Constant(-Prior::DefaultInverseVariance)));
}

TEST(ArgMin, ErrorTermBasePointer)
{
    using ET = ArgMin::ErrorTermBase<ArgMin::Scalar<double>, ArgMin::Dimension<2>, ArgMin::VariableGroup<SE3, SE3>>;

    SE3 pose;
    InverseDepth zinv;
    VariableContainer<SE3, InverseDepth> variableContainer;

    // insert variables
    auto se3Key1 = variableContainer.getVariableMap<SE3>().insert(pose);
    auto se3Key2 = variableContainer.getVariableMap<SE3>().insert(pose);

    auto dinvKey1 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);
    auto dinvKey2 = variableContainer.getVariableMap<InverseDepth>().insert(zinv);

    ET errorTerm;

    std::get<0>(errorTerm.variableKeys) = se3Key1;
    std::get<1>(errorTerm.variableKeys) = se3Key2;

    std::get<0>(errorTerm.variablePointers) = nullptr;
    std::get<1>(errorTerm.variablePointers) = nullptr;

    EXPECT_FALSE(errorTerm.checkVariablePointerConsistency(variableContainer));

    errorTerm.updateVariablePointers(variableContainer);

    EXPECT_TRUE(errorTerm.checkVariablePointerConsistency(variableContainer));
}

TEST(ArgMin, PSDSchurSolverSimple)
{
    SimpleScalar ss1 = 1;
    SimpleScalar ss2 = 2;
    DifferentSimpleScalar dss1 = 5;

    using ErrorTermSet = ArgMin::ErrorTermGroup<DifferenceErrorTerm>;

    using LS = ArgMin::PSDSchurSolver<Scalar<double>, ErrorTermSet, ArgMin::VariableGroup<SimpleScalar, DifferentSimpleScalar>, ArgMin::VariableGroup<InverseDepth>>;

    LS solver;

    VariableContainer<SimpleScalar, DifferentSimpleScalar> variableContainer;

    // insert variables
    auto ssKey1 = variableContainer.getVariableMap<SimpleScalar>().insert(ss1);
    auto ssKey2 = variableContainer.getVariableMap<SimpleScalar>().insert(ss2);

    auto dssKey1 = variableContainer.getVariableMap<DifferentSimpleScalar>().insert(dss1);

    ErrorTermContainer<DifferenceErrorTerm> errorTermContainer;

    // insert error terms
    auto errorTermKey1 = errorTermContainer.getErrorTermMap<DifferenceErrorTerm>().insert(DifferenceErrorTerm(ssKey1, dssKey1));
    auto errorTermKey2 = errorTermContainer.getErrorTermMap<DifferenceErrorTerm>().insert(DifferenceErrorTerm(ssKey2, dssKey1));

    // Test that reset runs
    solver.reset();

    // Test that initialize runs
    solver.initialize(variableContainer, errorTermContainer);
}