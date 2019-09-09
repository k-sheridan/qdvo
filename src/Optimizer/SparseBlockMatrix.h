#pragma once

#include <tuple>
#include <utility>
#include <memory>
#include <type_traits>
#include <vector>

#include "Key.h"
#include "MetaHelpers.h"
#include "SubMatrix.h"


namespace LittleOptimizer::SparseBlockMatrix {

namespace internal {

    /// Lowest level. add together the individual matrix blocks.
    template <typename T>
    bool add_rhs_column_matrices_to_lhs_column_matrices(std::vector<T>& lhsColumnMatrices, std::vector<T>& rhsColumnMatrices){
        assert(lhsColumnMatrices.size() == rhsColumnMatrices.size());

        for (size_t idx = 0; idx < lhsColumnMatrices.size(); ++idx){
            
        }
        return true;
    }

    template <typename T, size_t...Is, typename... Variables>
    bool add_rhs_row_to_lhs_row(std::vector<T>& lhsRow, std::vector<T>& rhsRow, std::integer_sequence<size_t, Is...>, VariableGroup<Variables...> vars){
        assert(lhsRow.size() == rhsRow.size());

        for (size_t idx = 0; idx < lhsRow.size(); ++idx){
            auto l = {add_rhs_column_matrices_to_lhs_column_matrices(std::get<Is>(lhsRow.at(idx).columns), std::get<Is>(rhsRow.at(idx).columns))...};
            (void)l;
        }

        return true;
    }

    template <typename T, size_t...Is, typename... Variables>
    void add_rhs_matrix_to_lhs_matrix(T& lhsMatrix, T& rhsMatrix, std::integer_sequence<size_t, Is...>, VariableGroup<Variables...> vars){
        auto l = {add_rhs_row_to_lhs_row(std::get<Is>(lhsMatrix), std::get<Is>(rhsMatrix), std::index_sequence_for<Variables...>{}, vars)...};
        (void)l;
    }

    // sets the size of a target vector to the size of a reference vector.
    template <typename T1>
    bool resize_vector(std::vector<T1>& theVector, size_t newSize) {
        theVector.resize(newSize);
        return true;
    }

    template <typename TupleOfVectors, size_t...Is>
    bool clear_tuple_of_vectors(TupleOfVectors& tupleOfVectors, std::integer_sequence<size_t, Is...>) {
        auto l = {resize_vector(std::get<Is>(tupleOfVectors), 0)...};
        (void)l;
        return true;
    }

    template <typename T>
    bool set_size(std::pair<T, size_t>& typeSizePair, size_t newSize) {
        typeSizePair.second = newSize;
        return true;
    }

    template <typename TupleOfTypeSizePairs, size_t...Is>
    bool set_sizes_zero(TupleOfTypeSizePairs& tuple, std::integer_sequence<size_t, Is...>) {
        auto l = {set_size(std::get<Is>(tuple), 0)...};
        (void)l;
        return true;
    }

};

template<typename... T>
class Row;

template <typename Scalar_, size_t Rows_, typename... Variables>
class Row<Scalar<Scalar_>, Dimension<Rows_>, VariableGroup<Variables...>> {
    public:
    typedef std::tuple<std::vector<SubMatrix<Scalar_, Rows_, Variables::dimension>>...> columns_t;
    
    //static_assert(std::is_same<std::tuple_size<columns_t>, decltype(sizeof...(Variables))>::value);

    columns_t columns;

    Row() {

    }

    // Expects: tuple<pair<T1, size_t>, pair<T2, size_t>, ...>
    // Sets up the columns to be a certiain size.
    template <typename TupleOfTypeSizePair, size_t...Is>
    Row(TupleOfTypeSizePair& tuple, std::integer_sequence<size_t, Is...>) {
        auto l = {internal::resize_vector(std::get<Is>(columns), std::get<Is>(tuple).second)...};
        (void)l;
    }
};

template <typename... Variables>
class SparseBlockMatrix;

template <typename Scalar_, typename... Variables>
class SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>> {

private:
    using variable_list = std::tuple<Variables...>;
    static_assert(sizeof...(Variables) > 0); // ensure that there is at least on variable.

    template <typename VariableType>
    using variable_index = LittleOptimizer::internal::Index<VariableType, variable_list>;

    typedef std::tuple<std::pair<Variables, size_t>...> sizes_t;
    sizes_t rowSizes, columnSizes; // Stores the size of the sub matrices for all type combintations.

    template <size_t dimension>
    using row_t = Row<Scalar<Scalar_>, Dimension<dimension>, VariableGroup<Variables...>>;

    typedef std::tuple< std::vector< row_t<Variables::dimension> >...> matrix_t;

    matrix_t matrix;

public:

    SparseBlockMatrix(){
        // resize everything to zero.
        this->clear();
    }

    /**
     * Removes all matrices and sets the total matrix size to 0.
     */
    void clear() {
        internal::clear_tuple_of_vectors(matrix, std::index_sequence_for<Variables...>{});
        internal::set_sizes_zero(rowSizes, std::index_sequence_for<Variables...>{});
        internal::set_sizes_zero(columnSizes, std::index_sequence_for<Variables...>{});
    }

    /**
     * Gets the sub matrix for the corresponding type keys, and returns it by reference.
     */
    template <typename RowType, typename ColType>
    SubMatrix<Scalar_, RowType::dimension, ColType::dimension>& get(LittleOptimizer::TypedIndex<RowType> row, LittleOptimizer::TypedIndex<ColType> col) {
        return std::get<variable_index<ColType>::value>(getRow(row).columns).at(col.index);
    }

    /**
     * Finds the row of the corresponding type key, and returns it by reference.
     */
    template <typename RowType>
    Row<Scalar<Scalar_>, Dimension<RowType::dimension>, VariableGroup<Variables...>>& getRow(LittleOptimizer::TypedIndex<RowType> row) {
        return std::get<variable_index<RowType>::value>(matrix).at(row.index);
    }

    /**
     * Computes how many rows there are for a given type.
     */
    template <typename RowType>
    size_t rowsOfType() {
        return std::get<std::pair<RowType, size_t>>(rowSizes).second;
    }

    /**
     * Computes how many rows there are for a given type.
     */
    template <typename ColumnType>
    size_t columnsOfType() {
        return std::get<std::pair<ColumnType, size_t>>(columnSizes).second;
    }

    /**
     * Adds a row of the correct size to the corresponding vector.
     */
    template <typename RowType>
    void addRow(){
        constexpr auto idx = variable_index<RowType>::value;
        static_assert(std::is_same<typename std::tuple_element<idx, sizes_t>::type, std::pair<RowType, size_t>>::value);
        assert(std::get<idx>(rowSizes).second == std::get<idx>(matrix).size());

        std::get<idx>(matrix).emplace_back(columnSizes, std::index_sequence_for<Variables...>{});
        std::get<idx>(rowSizes).second = std::get<idx>(matrix).size(); // update the row count.
    }

    /**
     * Adds together in the most efficient way two sparse block matrices.
     */
    SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>>& operator+=(SparseBlockMatrix<Scalar<Scalar_>, VariableGroup<Variables...>>& rhs){
        internal::add_rhs_matrix_to_lhs_matrix(matrix, rhs.matrix, std::index_sequence_for<Variables...>{}, VariableGroup<Variables...>());
        return *this;
    }

};

} // namespace LittleOptimizer