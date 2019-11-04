#pragma once

#include "MetaHelpers.h"
#include "Key.h"
#include "Containers.h"
#include "GaussianPrior.h"

namespace ArgMin
{

template <typename...>
class SSEOptimizer;

template <typename ScalarType, typename... Variables, typename... ErrorTerms>
class SSEOptimizer<Scalar<ScalarType>, VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>>
{
public:
    /// Inserts a variable into the optimizer.
    template <typename VariableType>
    VariableKey<VariableType> addVariable(VariableType &optimizableVariable)
    {
        return variables.template getVariableMap<VariableType>().insert(optimizableVariable);
    }

    // Updates the marginal information of the given variable.
    template <typename VariableType>
    void setVariablePrior(VariableKey<VariableType> &variableKey, Eigen::Matrix<ScalarType, VariableType::dimension, VariableType::dimension> &covariance)
    {
        assert(false);
    }

    /// Removes a variable and it's associated error terms, while approximating the 
    /// information they provided to the remaining variables with a gaussian prior.
    template <typename VariableType>
    void marginalizeVariable(VariableKey<VariableType>& key)
    {
        assert(false);
    }

    /// Removes a variable from the problem without marginalizing it.
    template <typename VariableType>
    void removeVariable(VariableKey<VariableType>& key)
    {
        variables.template getVariableMap<VariableType>().erase(key);

        // TODO erase the variables from the sparse prior.
    }

    /// Adds error term to the problem.
    template <typename ErrorTermType>
    ErrorTermKey<ErrorTermType> addErrorTerm(ErrorTermType& errorTerm)
    {
        assert(false);
    }

    /// Removes error term from the problem.
    template <typename ErrorTermType>
    void removeErrorTerm(ErrorTermKey<ErrorTermType>& errorTermKey)
    {
        assert(false);
    }

    /// Iteratively refines the variables until the SSE of the error terms is minimized.
    void optimize()
    {
        assert(false);
    }

private:
    /// Containers for both variables and error terms.
    VariableContainer<Variables...> variables;
    ErrorTermContainer<ErrorTerms...> errorTerms;

    /// Gaussian Prior
    GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> prior;
};

} //namespace ArgMin