#pragma once

#include <map>
#include <tuple>
#include <Eigen/Core>
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/SlotMap.h"

namespace ArgMin
{

template <typename...>
class SparseBlockRow;

template <typename ScalarType, typename... Variables>
class SparseBlockRow<ScalarType, VariableGroup<Variables...>>
{
private:
    template <typename VariableType>
    using Columns = std::map<VariableKey<VariableType>, >
public:
    SparseBlockRow(){};
};

} // namespace ArgMin