#pragma once

#include <Eigen/Core>
#include "Optimizer/Containers.h"
#include "Optimizer/SparseBlockMatrix.h"
#include "Optimizer/SparseBlockRow.h"
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
    using SBM = SparseBlockMatrix<Scalar<ScalarType>, VariableGroup<Variables...>>;
    using SBV = SparseBlockRow<Scalar<ScalarType>, Dimension<1>, VariableGroup<Variables...>>;

    SBM A0; // Sparse Block Information Matrix.
    SBV b0; // Sparse mean vector of the gaussian prior.

    GaussianPrior () {}

    /// Adds variable to the gaussian prior with an initial uncertainty.
    template <typename VariableType>
    void addVariable(VariableKey<VariableType>& key, Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension>& informationMatrix)
    {

    }

    /// Removes a variable from the 
    template <typename VariableType>
    void removeVariable(VariableKey<VariableType>& removedKey)
    {

    }

    // Updates the prior error term on manifold. A0 * (x + dx) = b0 => A0 * x = b0 - A0 * dx;
    void update(VariableContainer& variableOrder, Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& dx)
    {

    }

    
};

} // namespace ArgMin
