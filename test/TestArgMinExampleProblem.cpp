#include "Optimizer/MetaHelpers.h"
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
#include "Optimizer/HuberLossFunction.h"
#include <type_traits>

#include <gtest/gtest.h>

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

        information.setIdentity();
    }
};

class RelativeReprojectionError : public ErrorTermBase<Scalar<double>, Dimension<2>, VariableGroup<ArgMin::SE3, ArgMin::SE3, ArgMin::InverseDepth>> {

public:
    Eigen::Vector2d bearing;

    RelativeReprojectionError(VariableKey<ArgMin::SE3> hostFrame, VariableKey<ArgMin::SE3> targetFrame, VariableKey<ArgMin::InverseDepth> dinv, Eigen::Vector2d bearingMeasurement) {
        std::get<0>(variableKeys) = hostFrame;
        std::get<1>(variableKeys) = targetFrame;
        std::get<2>(variableKeys) = dinv;
        bearing = bearingMeasurement;
        information.setIdentity();
    }

    template <typename... Variables>
    void evaluate(VariableContainer<Variables...> &variables, bool relinearize)
    {
        EXPECT_TRUE(checkVariablePointerConsistency(variables));

        Sophus::SE3d &host = std::get<0>(variablePointers)->value;
        Sophus::SE3d &target = std::get<1>(variablePointers)->value;
        double &inverseDepth = std::get<2>(variablePointers)->value;

        // Compute the residual.

        if (relinearize)
        {
            Eigen::Matrix<double, 2, 6> &hostJacobian = (std::get<0>(variableJacobians));
            Eigen::Matrix<double, 2, 6> &targetJacobian = (std::get<1>(variableJacobians));
            Eigen::Matrix<double, 2, 1> &dinvJacobian = (std::get<2>(variableJacobians));

            // Compute the jacobians.

            linearizationValid = true;
        }
        else
        {
            linearizationValid = false;
        }
    }
};

/**
 * Set up a basic SLAM problem with 4 variable types. 2 correlated and 2 uncorrelated.
 */
class PSDSchurSolverTest : public ::testing::Test {
 protected:
  /**
   * Builds the solver with the current variables and errorterms.
   * This should only be ran once per test.
   */
  void SetUpProblem() {
     hostKey = variableContainer.insert(hostPose);
     targetKey = variableContainer.insert(targetPose);

     l1Key = variableContainer.insert(landmark1);
     l2Key = variableContainer.insert(landmark2);
     l3Key = variableContainer.insert(landmark3);

     ssKey = variableContainer.insert(unobservableScalar);
     dssKey = variableContainer.insert(uncorrelatedScalar);

     prior.addVariable(hostKey);
     prior.addVariable(targetKey);
     prior.addVariable(l1Key);
     prior.addVariable(l2Key);
     prior.addVariable(l3Key);
     prior.addVariable(ssKey);
     prior.addVariable(dssKey);

  }

  ArgMin::GaussianPrior<ArgMin::Scalar<double>, ArgMin::VariableGroup<ArgMin::SE3, ArgMin::InverseDepth, ArgMin::SimpleScalar, DifferentSimpleScalar>> prior;

  ArgMin::VariableContainer<ArgMin::SE3, ArgMin::InverseDepth, ArgMin::SimpleScalar, DifferentSimpleScalar> variableContainer;

  ArgMin::ErrorTermContainer<RelativeReprojectionError, DifferenceErrorTerm> errorTermContainer;

  ArgMin::HuberLossFunction<double> lossFunction = ArgMin::HuberLossFunction<double>(1000);

  using Solver = ArgMin::PSDSchurSolver<Scalar<double>, ArgMin::LossFunction<ArgMin::HuberLossFunction<double>>, ArgMin::ErrorTermGroup<RelativeReprojectionError, DifferenceErrorTerm>, ArgMin::VariableGroup<ArgMin::SE3, ArgMin::InverseDepth, ArgMin::SimpleScalar, DifferentSimpleScalar>, ArgMin::VariableGroup<ArgMin::InverseDepth, DifferentSimpleScalar>>;

  Solver solver = Solver(lossFunction);

  // correlated variables.
  ArgMin::SE3 hostPose;
  ArgMin::SE3 targetPose;
  ArgMin::SimpleScalar unobservableScalar = ArgMin::SimpleScalar(10);
  // uncorrelated variables.
  DifferentSimpleScalar uncorrelatedScalar = DifferentSimpleScalar(20);
  ArgMin::InverseDepth landmark1 = ArgMin::InverseDepth(1);
  ArgMin::InverseDepth landmark2 = ArgMin::InverseDepth(0.75);
  ArgMin::InverseDepth landmark3 = ArgMin::InverseDepth(0.5);

  VariableKey<SE3> hostKey, targetKey;
  VariableKey<InverseDepth> l1Key, l2Key, l3Key;
  VariableKey<SimpleScalar> ssKey;
  VariableKey<DifferentSimpleScalar> dssKey;

};

TEST_F(PSDSchurSolverTest, IterateWithOnlyPrior) {
    SetUpProblem();

    solver.initialize(variableContainer, errorTermContainer);
    solver.linearize(variableContainer, errorTermContainer);
    solver.buildLinearSystem(prior, errorTermContainer);
    solver.solveLinearSystem(variableContainer, errorTermContainer, prior);

    // Since the prior is the only constraint, the dx vector should be ~0;
    EXPECT_NEAR(solver.dx.block(0, 0, solver.totalDimension, 1).norm(), 0, 1e-9);
    int previousDimension = solver.totalDimension;

    //Remove a variable and compute another iteration.
    variableContainer.erase(targetKey);
    prior.removeUnsedVariables(variableContainer);

    // Verify that the row was erased.
    EXPECT_EQ(prior.A0.getRowMap<ArgMin::SE3>().count(targetKey), 0);

    // Run another iteration from scratch and verify everything still works as expected.
    solver.initialize(variableContainer, errorTermContainer);
    solver.linearize(variableContainer, errorTermContainer);
    solver.buildLinearSystem(prior, errorTermContainer);
    solver.solveLinearSystem(variableContainer, errorTermContainer, prior);

    EXPECT_EQ(solver.totalDimension, previousDimension - ArgMin::SE3::dimension);
    EXPECT_NEAR(solver.dx.block(0, 0, solver.totalDimension, 1).norm(), 0, 1e-9);
}