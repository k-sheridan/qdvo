#pragma once

#include "Optimizer/SlotMap.h"
#include "Optimizer/Key.h"
#include "Optimizer/MetaHelpers.h"
#include <Eigen/Core>
#include <tuple>

namespace ArgMin {

template <typename... T>
class BlockVector;

/**
 * Unlike the Sparse Block Row, the BlockVector is assumed to be dense and can then 
 * achieve constant time insert, lookup, and erase via the slot map.
 * This class uses a slightly different variant of the slot map. One which
 * allows us to choose a key value pair.
 */
template <typename ScalarType, int ColumnDimension, typename... Variables>
class BlockVector<Scalar<ScalarType>, Dimension<ColumnDimension>, VariableGroup<Variables...>> {
public:
    template <typename VariableType>
    using MatrixBlock = Eigen::Matrix<ScalarType, VariableType::dimension, ColumnDimension>;
    template <typename VariableType>
    using RowMap = SlotMap<MatrixBlock<VariableType>, VariableKey<VariableType>>;

    BlockVector() {}

    /// Inserts a key value pair into the slot map.
    /// This should never need overwrite an element. 
    template <typename VariableType>
    void addRowBlock(VariableKey<VariableType> key, MatrixBlock<VariableType>& value)
    {
        // Get key used to access the internal slot map.
        auto& internalKey = getInternalKey(key);

        if (internalKey.isInvalid())
        {
            // If the internal key is invalid, insert a new element into the slot map.
            internalKey = std::get<RowMap<VariableType>>(tupleOfRowMaps).insert(value);
        }
        else
        {
            // If there is a valid key already, overwrite the old element.
            auto it = std::get<RowMap<VariableType>>(tupleOfRowMaps).at(internalKey);

            assert(it != std::get<RowMap<VariableType>>(tupleOfRowMaps).end());

            *it = value;
        }
        
    }

    /// Removes the row block.
    template <typename VariableType>
    void removeRowBlock(VariableKey<VariableType> key)
    {
        // Get key used to access the internal slot map.
        auto& internalKey = getInternalKey(key);

        // Erase the slot map element at the internal key.
        std::get<RowMap<VariableType>>(tupleOfRowMaps).erase(internalKey);

        internalKey.setInvalid();
    }

    /// Gets a reference to the block matrix at the key.
    template <typename VariableType>
    MatrixBlock<VariableType>& getRowBlock(VariableKey<VariableType> key)
    {
        // Get key used to access the internal slot map.
        auto& internalKey = getInternalKey(key);

        assert(internalKey.isInvalid() != true);

        // Get the element at the internal key.
        auto it = std::get<RowMap<VariableType>>(tupleOfRowMaps).at(internalKey);

        assert(it != std::get<RowMap<VariableType>>(tupleOfRowMaps).end());

        return *it;
    }

    /// Evaluates if the key exists in the block vector.
    template <typename VariableType>
    bool blockExists(VariableKey<VariableType> key)
    {
        // Get key used to access the internal slot map.
        auto& internalKey = getInternalKey(key);

        return !internalKey.isInvalid();
    }

    /// Computes BlockVector -= v.
    void subtractVector(VariableContainer<Variables...> &variableOrder, const Eigen::Matrix<ScalarType, Eigen::Dynamic, ColumnDimension> &v)
    {
        internal::static_for(variableOrder.tupleOfVariableMaps, [&](auto i, auto &variableMap){
            auto &map = variableOrder.template getVariableMap<typename std::tuple_element<i, std::tuple<Variables...>>::type>();
            constexpr int variableDimension = std::tuple_element<i, std::tuple<Variables...>>::type::dimension;

            MatrixBlock<typename std::tuple_element<i, std::tuple<Variables...>>::type> zero = MatrixBlock<typename std::tuple_element<i, std::tuple<Variables...>>::type>::Zero();

            if (map.size() > 0)
            {
                auto firstVariableKey = map.getKeyFromDataIndex(0);
                // Precompute the offset index for the current variable type.
                auto startingIndex = variableOrder.template variableIndex(firstVariableKey);

                for (auto variableIterator = map.begin(); variableIterator != map.end(); variableIterator++)
                {
                    auto key = map.getKeyFromDataIndex(variableIterator - map.begin());
                    int index = startingIndex + (variableIterator - map.begin()) * variableDimension;

                    const auto& rhs = v.block(index, 0, variableDimension, 1);

                    if (blockExists(key))
                    {
                        auto& rowBlock = getRowBlock(key);

                        rowBlock -= rhs;
                    }
                    else
                    {
                        // Add the row.
                        addRowBlock(key, zero);
                        getRowBlock(key) -= rhs;
                    }

                }
            }
        });
    }

private:

    /// Gets the internal slot map key for a given external key.
    /// If the KeyIndexMap is not large enough, it will be resized.
    template <typename VariableType>
    VariableKey<VariableType>& getInternalKey(VariableKey<VariableType>& externalKey)
    {
        assert(!externalKey.isInvalid());

        auto& keyIndexMap = std::get<std::vector<VariableKey<VariableType>>>(tupleOfKeyIndexMaps);

        if (externalKey.index >= keyIndexMap.size())
        {
            VariableKey<VariableType> invalidKey;
            invalidKey.setInvalid();

            keyIndexMap.resize(externalKey.index + 1, invalidKey);

        }

        return keyIndexMap.at(externalKey.index);
    }

std::tuple<RowMap<Variables>...> tupleOfRowMaps;

std::tuple<std::vector<VariableKey<Variables>>...> tupleOfKeyIndexMaps; // maps desired key index to slotmap index.

};

} // namespace ArgMin
