#pragma once

#include <Eigen/Core>
#include "Optimizer/Containers.h"
#include "Optimizer/SparseBlockMatrix.h"
#include "Optimizer/SparseBlockRow.h"
#include "Optimizer/BlockVector.h"
#include "Optimizer/Key.h"
#include "MetaHelpers.h"

namespace ArgMin
{

template <typename... T>
class GaussianPrior;

template <typename ScalarType, typename... Variables>
class GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>>
{
public:
    static constexpr ScalarType DefaultInverseVariance = 1e-24;

    using SBM = SparseBlockMatrix<Scalar<ScalarType>, VariableGroup<Variables...>>;

    using BV = BlockVector<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>>;

    SBM A0; // Sparse Block Information Matrix.

    BV b0; // dense mean vector of the gaussian prior.

    Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> temporaryVector; // Preallocated vector.

    GaussianPrior() {}

    /// Adds variable to the gaussian prior with an initial uncertainty.
    template <typename VariableType>
    void addVariable(VariableKey<VariableType> &key, Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension> informationMatrix = Eigen::Matrix<ScalarType, VariableType::dimension, 1>::Constant(DefaultInverseVariance).asDiagonal())
    {
        // insert the information block
        A0.setBlock(key, key, informationMatrix);

        // Insert a zero mean.
        Eigen::Matrix<ScalarType, VariableType::dimension, 1> zeroVec = Eigen::Matrix<ScalarType, VariableType::dimension, 1>::Zero();
        b0.addRowBlock(key, zeroVec);
    }

    /// Removes a variable from the
    template <typename VariableType>
    void removeVariable(VariableKey<VariableType> &removedKey)
    {
    }

    // Updates the prior error term on manifold. A0 * (x + dx) = b0 => A0 * x = b0 - A0 * dx;
    void update(VariableContainer<Variables...> &variableOrder, const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> &dx)
    {
        
        // Uneccesarily expensive. This can be packaged directly into the dot function.
        // This is ran once per iteration.
        size_t problemSize = variableOrder.totalDimensions();
        assert(dx.rows() == problemSize); // ensure that dx is consistent with the current variable set.

        if (problemSize > temporaryVector.rows())
        {
            temporaryVector.resize(problemSize, Eigen::NoChange);
        }

        // Compute the perturbation.
        A0.dot(variableOrder, dx, temporaryVector);

        // Move the mean.
        b0.subtractVector(variableOrder, temporaryVector);
    }
};

} // namespace ArgMin
