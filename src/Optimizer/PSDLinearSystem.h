#pragma once

#include "Containers.h"
#include "MetaHelpers.h"
#include "GaussianPrior"

template <typename... T>
class PSDLinearSystem;

template <typename ScalarType, typename... ErrorTerms, typename... Variables>
class PSDLinearSystem<Scalar<ScalarType>, ErrorTermGroup<ErrorTerms...>, VariableGroup<Variables...>> {

};