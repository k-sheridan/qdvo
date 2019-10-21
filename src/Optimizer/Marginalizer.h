#pragma once

#include "Optimizer/GaussianPrior.h"
#include "Optimizer/Key.h"
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/Containers.h"

namespace ArgMin {

template <typename... T>
class Marginalizer;

template <typename ScalarType, typename... Variables, typename... ErrorTerms>
class Marginalizer<Scalar<ScalarType>, VariableGroup<Variables...>, ErrorTermGroup<ErrorTerms...>> {

public:

    Marginalizer() {}

    /// Marginalizes the variable requested using a set of linearized error terms.
    template <typename VariableType>
    void marginalizeVariable(VariableKey<VariableType>& marginalizedKey, GaussianPrior<Scalar<ScalarType>, VariableGroup<Variables...>> ErrorTermContainer<ErrorTerms...>& linearizedErrorTerms)
    {

    }

}

};