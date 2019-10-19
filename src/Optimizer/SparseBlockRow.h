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

template <typename ScalarType, int RowDimension, typename... Variables>
class SparseBlockRow<Scalar<ScalarType>, Dimension<RowDimension>, VariableGroup<Variables...>>
{
public:
    template <typename VariableType>
    using MatrixBlock = Eigen::Matrix<ScalarType, RowDimension, VariableType::dimension>;

    template <typename VariableType>
    using VariableColumns = std::map<VariableKey<VariableType>, MatrixBlock<VariableType>>;

    SparseBlockRow() {}

    /// Returns a std map of blocks for the given variable type.
    template <typename VariableType>
    VariableColumns<VariableType> &getVariableMap()
    {
        return std::get<VariableColumns<VariableType>>(columns);
    }

    /// Computes the dot product of this row with a dense column vector.
    /// The variable container is used to determine the indices of each block.
    Eigen::Matrix<ScalarType, RowDimension, 1> dot(const VariableContainer<Variables...> &variableOrder, const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> &v)
    {
    }

private:
    std::tuple<VariableColumns<Variables>...> columns; // Main storage for the matrix blocks in the row.
};

} // namespace ArgMin