#pragma once

#include <map>
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/SparseBlockMatrix.h"

namespace ArgMin
{

template <typename...>
class SparseBlockMatrix;

template <typename ScalarType, typename... Variables>
class SparseBlockMatrix<Scalar<ScalarType>, VariableGroup<Variables...>>
{

};

} // namespace ArgMin