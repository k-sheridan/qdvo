#pragma once

#include <map>
#include <tuple>
#include <Eigen/Core>
#include "Optimizer/MetaHelpers.h"
#include "Optimizer/SlotMap.h"
#include "Optimizer/Containers.h"

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
    Eigen::Matrix<ScalarType, RowDimension, 1> dot(VariableContainer<Variables...> &variableOrder, const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> &v)
    {
        Eigen::Matrix<ScalarType, RowDimension, 1> result = Eigen::Matrix<ScalarType, RowDimension, 1>::Zero();

        internal::static_for(columns, [&](auto i, auto &matrixMap) {
            auto &map = variableOrder.template getVariableMap<typename std::tuple_element<i, std::tuple<Variables...>>::type>();

            if (map.size() > 0)
            {
                auto firstVariableKey = map.getKeyFromDataIndex(0);
                // Precompute the offset index for the current variable type.
                auto startingIndex = variableOrder.template variableIndex(firstVariableKey);

                for (auto &pair : matrixMap)
                {
                    auto variableIterator = map.at(pair.first);
                    if (variableIterator != map.end())
                    {
                        auto index = startingIndex + (variableIterator - map.begin()) * std::tuple_element<i, std::tuple<Variables...>>::type::dimension;

                        result += pair.second * v.block(index, 0, std::tuple_element<i, std::tuple<Variables...>>::type::dimension, 1);
                    }
                }
            }
        });

        return result;
    }

    /// Sets all current non zero blocks to zero.
    void setZero()
    {
        internal::static_for(columns, [](auto i, auto& v) {
            for (auto& pair : v)
            {
                pair.second.setZero();
            }
        });
    }

private:
    std::tuple<VariableColumns<Variables>...> columns; // Main storage for the matrix blocks in the row.
};

} // namespace ArgMin