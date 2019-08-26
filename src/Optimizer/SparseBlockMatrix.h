#pragma once

#include <tuple>
#include <type_traits>
#include <vector>
#include <Eigen/Core>

#include "Key.h"


namespace LittleOptimizer::SparseBlockMatrix {

template <typename... T>
struct VariableGroup {};

template <typename... T>
class Row;

template <int T>
class Dimension;

template <typename T>
class Scalar;

template <typename Scalar_, int Rows_, typename... Variables>
class Row<Scalar<Scalar_>, Dimension<Rows_>, VariableGroup<Variables...>> {
    typedef std::tuple<std::vector<Eigen::Matrix<Scalar_, Rows_, Variables::dimension>>...> columns_t;
    
    static_assert(std::is_same<std::tuple_size<columns_t>, decltype(sizeof...(Variables))>::value);

    columns_t columns;
};

template <typename... Variables>
class SparseBlockMatrix;

template <typename Scalar_, typename... Variables>
class SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>> {

    using matrix_t = std::tuple< std::vector<Row<Scalar_, decltype(Variables::dimension), VariableGroup<Variables...>> >...>;

    static_assert(std::is_same<std::tuple_size<matrix_t>, decltype(sizeof...(Variables))>::value);
    static_assert(std::is_same<std::tuple_size<matrix_t>, std::tuple_size<decltype(matrix_t::columns)>>::value);

    matrix_t matrix;

    template <typename RowType, typename ColType>
    Eigen::Matrix<Scalar_, RowType::dimension, ColType::dimension>& get(LittleOptimizer::VariableKey<RowType> row, LittleOptimizer::VariableKey<ColType> col) {
        return std::get<std::vector<Eigen::Matrix<Scalar_, RowType::dimension, ColType::dimension>>>(getRow(row).columns).at(col.index);
    }

    template <typename RowType>
    Row<Scalar_, decltype(RowType::dimension), VariableGroup<Variables...>> getRow(LittleOptimizer::VariableKey<RowType> row) {
        return std::get<std::vector<Row<Scalar_, decltype(RowType::dimension), VariableGroup<Variables...>> >>(matrix).at(row.index);
    }

};

} // namespace LittleOptimizer