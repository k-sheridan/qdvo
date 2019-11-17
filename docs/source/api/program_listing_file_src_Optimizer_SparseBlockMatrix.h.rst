
.. _program_listing_file_src_Optimizer_SparseBlockMatrix.h:

Program Listing for File SparseBlockMatrix.h
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_Optimizer_SparseBlockMatrix.h>` (``src/Optimizer/SparseBlockMatrix.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

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
   
       template <typename VariableType>
       RowMap<VariableType>& getRowMap()
       {
           return std::get<RowMap<VariableType>>(tupleOfRowMaps);
       }
   
       template <typename VariableType>
       auto& addRowIfItDoesNotExist(VariableKey<VariableType> key)
       {
           // Only insert if the key is not in the map.
           auto rowIt = getRowMap<VariableType>().find(key);
   
           if (rowIt == getRowMap<VariableType>().end())
           {
               std::tie(rowIt, std::ignore) = getRowMap<VariableType>().insert(std::make_pair(key, Row<VariableType>()));
           }
   
           return rowIt->second;
       }
   
       template <typename RowType, typename ColType>
       void setBlock(VariableKey<RowType> rowKey, VariableKey<ColType> colKey, const Eigen::Matrix<ScalarType, RowType::dimension, ColType::dimension>& blockMatrix)
       {
           // Add a row to the sbm.
           auto& row = addRowIfItDoesNotExist(rowKey);
   
           // insert or assign the block
           row.template getVariableMap<ColType>().insert_or_assign(colKey, blockMatrix);
       }
   
       template <typename RowType, typename ColType>
       void removeBlock(VariableKey<RowType> rowKey, VariableKey<ColType> colKey)
       {
           // get the row.
           auto row = getRowMap<RowType>().find(rowKey);
   
           // If the row exists remove the column.
           if (row != getRowMap<RowType>().end())
           {
               row->second.template getVariableMap<ColType>().erase(colKey);
           }
           
       }
   
       
   
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
