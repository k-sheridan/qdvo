# Little Optimizer
## Overview

Little Optimizer is a semi generic nonlinear optimizer capable of performing Levenberg-Marquardt or Gauss-Newton. Little Optimizer also stores a gaussian prior for all variables in its state. 

Little Optimizer is written with a significant amount of template meta programming to improve speed. The key way high speed is achieved is by using Little Optimizer's Sparse Block Matrix/Vector classes for most operations including. These classes are built on top of Eigen using a set of fixed size matrices. This allows Little Optimizer to exploit Eigen's vectorized operations.

The application Little Optimizer was designed for is small SLAM problems. Specifically, optimizer was designed for windowed SLAM problems. The main reason the library is called "Little" Optimizer is because of the way the Sparse Block Matrix is implemented. The Sparse Block Matrix allocated memory for all matrix blocks regardless of their sparsity. This allows for constant time access to all matrix blocks and a flag stating whether the matrix is zero or not.

```cpp
class LittleOptimizer <VariableGroup<V1, V2, ...>, ErrorTermGroup<E1, E2, ...>> {

public:

// Adds a variable to the problem. The returned variable key should be stored by the user for future use.
VariableKey<V> addVariable(V optimizableVariable);

// Removes a variable and it's associated error terms, while approximating the 
// information they provided to the remaining variables with a gaussian prior.
void marginalizeVariable(VariableKey<V> key);

// Removes a variable from the problem without marginalizing it.
void removeVariable(VariableKey<V> key);

// Adds error term to the problem.
void addErrorTerm(E errorTerm);

// Iteratively refines the variables until the SSE of the error terms is minimized.
void optimize();

private:

ErrorTermContainers errorTerms; // Tuple of vectors of all error term types.
VariableContainers variables; // Tuple of vectors of variables.

PSDLinearSystem linearSystem; // Used to solve for perturbation.
GaussianPrior prior; // Used during marginalization to approximate deleted information.

// Sets up the linear system efficiently. (A_prior + Jt * W^(-1) * J) * dx = (b_prior + Jt * W^(-1) * e).
void buildProblem();

// Solves the linear system efficiently using schur complement.
// The given index is used to find where the top left element of D is.
// [ A   |  B ]
// [ -------- ]
// [ Bt  |  D ]
// Where D is the block diagonal matrix.
void solveUsingSchurComplement(TypedIndex<T> blockDiagonalStartIndex);

};
```

## Variable and ErrorTerm Requirements
### Optimizable Variable Requirements
```cpp
class OptimizableVariable<Scalar, VariableDimension> {

// The dimension if the minimal form of the variable.
static int Dimension = VariableDimension;

// Variable Data or Reference to data.
Variable;

// Generalized plus operation. Allows for on manifold optimization.
static void update(Eigen::Matrix<Scalar, Dimension, 1> delta);

};
```

### Error Term Requirements
```cpp
class ErrorTermBase<ScalarType, VariableGroup<V1, V2, ...>, ResidualDimension> {

// Stores the keys to the variables this error term is a function of.
tuple<VariableKey<V1>, VariableKey<V2>, ...> variableKeys;

// Linearized error term.
{
  bool linearized = false; // Flag which marks whether the error term was ever linearized.

  // The point at which the error term was last linearized.
  tuple<V1, V2, ...> linearizationPoint;

  // Jacobians of the error term w.r.t each variable at the linearization point.
  tuple<Eigen::Matrix<ScalarType, ResidualDimension, V1::dimension>, Eigen::Matrix<ScalarType, ResidualDimension, V2::dimension>, ...> jacobians;

  // Residual/Error at the linearization point.
  Eigen::Matrix<ScalarType, ResidualDimension, 1> residual;
}
};

class ErrorTerm : public ErrorTermBase<Scalar, VariableGroup<V1, V2, ...>, ResidualDimension> {

// Computes the error given the current state of the variables, optionally linearizes the error function.
void computeResidual(std::tuple<V1*, V2*, ...> variables, bool linearize = false);

};
```

## Sparse Linear Equations

### Sparse Block Matrix
```cpp
class SparseBlockMatrix<ScalarType, VariableGroup<V1, V2, ...>> {
  
  // Gets a block matrix at a given index.
  MatrixBlock<ScalarType, T1::Dimension, T2::Dimension> get(TypedIndex<T1> row, TypedIndex<T2> col);

  // Adds a row to the matrix with the dimension of the given RowType
  TypedIndex<RowType> addRow<RowType>();

  // Adds a col to the matrix with the dimension of the given ColType
  TypedIndex<ColType> addColumn<ColType>();

  void removeRow(TypedIndex<RowType> row);
  void removeColumn(TypedIndex<ColType> col);

  // Copies the blocks between the indices into the dense sub matrix.
  void fillDenseSubMatrix(TypedIndex<T1> topRow, TypedIndex<T2> topColumn, TypedIndex<T3> bottomRow, TypedIndex<T4> bottomColumn, Eigen::Matrix<ScalarType>& denseMatrix);

  // Computes the matrix (non-block) index of the top left part of the matrix block using a TypedIndex<T>. 
  size_t computeRowIndex(TypedIndex<T> idx);
  size_t computeColumnIndex(TypedIndex<T> idx);

  // += operator is overloaded

  // * operator overloaded for Matrix * Row^(T)

};
```
### Sparse Block Row
```cpp
class SparseBlockRow<ScalarType, RowDimension, VariableGroup<V1, V2, ...>> {
  
  // Gets a block vector at a given index.
  MatrixBlock<ScalarType, RowDimension, T1::Dimension> get(TypedIndex<T1> row);

  // Adds a row to the matrix with the dimension of the given RowType
  TypedIndex<ColumnType> addColumn<ColType>();

  void removeColumn(TypedIndex<ColumnType> col);

  // Copies the block matrices between indices into the dense vector. row and column are where 
  // in the dense matrix the row is added to.
  void fillDenseSubMatrix(TypedIndex<T1> leftIndex, rightIndex<T2> bottomIndex, size_t row, size_t column, Eigen::Matrix<ScalarType>& denseMatrix);

  // += operator overloaded

  // * operator overloaded

};
```

### Positive Semi-Definite Linear System
```cpp
class PSDLinearSystem<ScalarType, VariableGroup<V1, V2, ...>> {

  SparseBlockMatrix A;
  SparseBlockVector x, b;

  // adds a variable to the linear system, and resizes the matrix and vector accordingly
  TypedIndex<T> addVariable<T>();

  // overloaded += operator which adds two PSDLinearSystems together: (A1 + A2) * x = (b1 + b2)

};
```

### Gaussian Prior
```cpp
class GaussianPrior : public PSDLinearSystem<ScalarType, VariableGroup<V1, V2, ...>> {

// Updates the prior error term on manifold. A * (x + dx) = b => A * x = b - A * dx; 
void update(SparseBlockVector<ScalarType, VariableGroup<V1, V2, ...>> dx);

};
```


