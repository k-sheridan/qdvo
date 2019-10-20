#pragma once

#include <map>
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/SparseBlockRow.h"

namespace ArgMin
{

template <typename...>
class SparseBlockMatrix;

template <typename ScalarType, typename... Variables>
class SparseBlockMatrix<Scalar<ScalarType>, VariableGroup<Variables...>>
{
public:
    template <typename VariableType>
    using Row = SparseBlockRow<Scalar<ScalarType>, Dimension<VariableType::dimension>, VariableGroup<Variables...>>;

    template <typename VariableType>
    using RowMap = std::map<VariableKey<VariableType>, Row<VariableType>>;

    SparseBlockMatrix() {}

    /// Gets the map of rows for a given variable type
    template <typename VariableType>
    RowMap<VariableType>& getRowMap()
    {
        return std::get<RowMap<VariableType>>(tupleOfRowMaps);
    }

    /// Computes the dot product of this Matrix with a dense column vector.
    /// The variable container is used to determine the indices of each block.
    /// a.k.a. A * v = result
    template <int DenseMatrixColumns>
    void dot(VariableContainer<Variables...> &variableOrder, const Eigen::Matrix<ScalarType, Eigen::Dynamic, DenseMatrixColumns> &v, Eigen::Matrix<ScalarType, Eigen::Dynamic, DenseMatrixColumns>& result)
    {
        // Not that efficient, but the result vector must be zero by default
        result.setZero();

        internal::static_for(tupleOfRowMaps, [&](auto i, auto &rowMap) {
            auto &map = variableOrder.template getVariableMap<typename std::tuple_element<i, std::tuple<Variables...>>::type>();

            assert(rowMap.size() <= map.size()); // There should be no more variables in the SBM than are in the variable container.

            if (map.size() > 0)
            {
                auto firstVariableKey = map.getKeyFromDataIndex(0);
                // Precompute the offset index for the current variable type.
                auto startingIndex = variableOrder.template variableIndex(firstVariableKey);

                Eigen::Matrix<ScalarType, std::tuple_element<i, std::tuple<Variables...>>::type::dimension, DenseMatrixColumns> blockResult;

                for (auto &pair : rowMap)
                {
                    auto variableIterator = map.at(pair.first);
                    if (variableIterator != map.end())
                    {
                        auto index = startingIndex + (variableIterator - map.begin()) * std::tuple_element<i, std::tuple<Variables...>>::type::dimension;

                        pair.second.dot(variableOrder, v, blockResult);
                        result.block(index, 0, std::tuple_element<i, std::tuple<Variables...>>::type::dimension, DenseMatrixColumns) = blockResult;
                    }
                    else {
                        assert(false && "variable in SBM not in variable container!");
                    }
                }
            }
        });
    }

    private:

    std::tuple<RowMap<Variables>...> tupleOfRowMaps;
};

} // namespace ArgMin