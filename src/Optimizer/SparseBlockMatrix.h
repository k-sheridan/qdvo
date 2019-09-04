#pragma once

#include <tuple>
#include <utility>
#include <memory>
#include <type_traits>
#include <vector>
#include <Eigen/Core>

#include "Key.h"


namespace LittleOptimizer::SparseBlockMatrix {

template <typename... T>
struct VariableGroup {};

template <int T>
struct Dimension {};

template <typename T>
struct Scalar {};

namespace internal {
    /// Lowest level. add together the individual matrix blocks.
    template <typename T, size_t...Is>
    void add_rhs_column_matrices_to_lhs_column_matrices(std::vector<std::unique_ptr<T>>& lhsColumnMatrices, std::vector<std::unique_ptr<T>>& rhsColumnMatrices, std::integer_sequence<size_t, Is...>){
        assert(lhsColumnMatrices.size() == rhsColumnMatrices.size());

        for (size_t idx = 0; idx < lhsColumnMatrices.size(); ++idx){
            
        }
    }

    template <typename T, size_t...Is, typename... Variables>
    void add_rhs_row_to_lhs_row(std::vector<T>& lhsRow, std::vector<T>& rhsRow, std::integer_sequence<size_t, Is...>, VariableGroup<Variables...> vars){
        assert(lhsRow.size() == rhsRow.size());

        for (size_t idx = 0; idx < lhsRow.size(); ++idx){
            auto l = {add_rhs_column_matrices_to_lhs_column_matrices(std::get<Is>(lhsRow.at(idx), rhsRow.at(idx), std::index_sequence_for<Variables...>{}, VariableGroup<Variables...>()))...};
            (void)l;
        }
    }

    template <typename T, size_t...Is, typename... Variables>
    void add_rhs_matrix_to_lhs_matrix(T& lhsMatrix, T& rhsMatrix, std::integer_sequence<size_t, Is...>, VariableGroup<Variables...> vars){
        auto l = {add_rhs_row_to_lhs_row(std::get<Is>(lhsMatrix), std::get<Is>(rhsMatrix), std::index_sequence_for<Variables...>{})...};
        (void)l;
    }
};

template<typename... T>
class Row;

template <typename Scalar_, int Rows_, typename... Variables>
class Row<Scalar<Scalar_>, Dimension<Rows_>, VariableGroup<Variables...>> {
    typedef std::tuple<std::vector<std::unique_ptr<Eigen::Matrix<Scalar_, Rows_, Variables::dimension>>>...> columns_t;
    
    //static_assert(std::is_same<std::tuple_size<columns_t>, decltype(sizeof...(Variables))>::value);

    columns_t columns;
};

template <typename... Variables>
class SparseBlockMatrix;

template <typename Scalar_, typename... Variables>
class SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>> {

    typedef std::tuple< std::vector<Row<Scalar<Scalar_>, Dimension<Variables::dimension>, VariableGroup<Variables...>> >...> matrix_t;

    matrix_t matrix;

    /*template <typename RowType, typename ColType>
    std::unique_ptr<Eigen::Matrix<Scalar_, RowType::dimension, ColType::dimension>>& get(LittleOptimizer::VariableKey<RowType> row, LittleOptimizer::VariableKey<ColType> col) {
        return std::get<std::vector<Eigen::Matrix<Scalar_, RowType::dimension, ColType::dimension>>>(getRow(row).columns).at(col.index);
    }

    template <typename RowType>
    Row<Scalar_, decltype(RowType::dimension), VariableGroup<Variables...>> getRow(LittleOptimizer::VariableKey<RowType> row) {
        return std::get<std::vector<Row<Scalar_, decltype(RowType::dimension), VariableGroup<Variables...>> >>(matrix).at(row.index);
    }

    SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>>& operator+=(const SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>>& rhs){
        internal::add_rhs_matrix_to_lhs_matrix(matrix, rhs.matrix, std::index_sequence_for<Variables...>{}, VariableGroup<Variables...>());
        return *this;
    }*/

};

} // namespace LittleOptimizer