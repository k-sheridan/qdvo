#pragma once

#include "Optimizer/GaussianPrior.h"
#include "Optimizer/Key.h"
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/Containers.h"

namespace ArgMin {

template <typename... T>
class Marginalizer;

/**
 * The marginalizer is a class which is responsible for approximating the effect
 * of a variable's error terms on the rest of the variables.
 * This is done by approximating all error terms involving the error terms 
 * with a single gaussian prior through the schur complement.
 */
template <typename ScalarType, typename... Variables, typename... ErrorTerms>
class Marginalizer<Scalar<ScalarType>, VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> {

public:

    Marginalizer() {}

    /// Marginalizes the variable requested using a set of linearized error terms.
    /// It will be assumed that the error term jacobians and residual are good 
    /// approximations of the error term.
    template <typename VariableType>
    void marginalizeVariable(VariableKey<VariableType>& marginalizedKey, GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> ErrorTermContainer<ErrorTerms...>& linearizedErrorTerms)
    {

    }

}

};