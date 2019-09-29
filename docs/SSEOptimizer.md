# SSEOptimizer
## Overview

SSEOptimizer is a semi generic nonlinear optimizer capable of performing Levenberg-Marquardt or Gauss-Newton on manifold. SSEOptimizer also stores a gaussian prior for all variables in its state. 

The application SSEOptimizer was designed for is small SLAM problems. Specifically, optimizer was designed for windowed SLAM problems. 

```cpp
class SSEOptimizer <VariableGroup<V1, V2, ...>, ErrorTermGroup<E1, E2, ...>> {

public:

// Adds a variable to the problem. The returned variable key should be stored by the user for future use.
VariableKey<V> addVariable(V optimizableVariable);

// Removes a variable and it's associated error terms, while approximating the 
// information they provided to the remaining variables with a gaussian prior.
void marginalizeVariable(VariableKey<V> key);

// Removes a variable from the problem without marginalizing it.
void removeVariable(VariableKey<V> key);

// Adds error term to the problem.
ErrorTermKey<E> addErrorTerm(E errorTerm);

// Removes error term from the problem.
void removeErrorTerm(ErrorTermKey<E> errorTermKey);

// Iteratively refines the variables until the SSE of the error terms is minimized.
void optimize();

private:

std::shared_ptr<ErrorTermContainer<E1, ...>> errorTerms; // stores all error term types.
std::shared_ptr<VariableContainer<V1, ...>> variables; // stores all variables.

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
## Variable and ErrorTerm Storage
template <Variables...>
struct VariableContainer : tuple<slot_map<V1>, ...> {
   // Gets the index of the first scalar of the given variable. 
   // This is used to build and operate on a matrix.
   size_t variableIndex(VariableKey<V> key);
};
  
template <ErrorTerms...>
struct ErrorTermContainer : tuple<slot_map<E1>, ...> {

};

## Solving for the perturbation.
### Positive Semi-Definite Linear System
```cpp
class PSDLinearSystem<ScalarType, VariableGroup<V1, V2, ...>> {

  Matrix A;
  Vector x, b;

  // adds a variable to the linear system, and resizes the matrix and vector accordingly
  TypedIndex<T> addVariable<T>();

  // overloaded += operator which adds two PSDLinearSystems together: (A1 + A2) * x = (b1 + b2)

};
```

## Marginalization
### Gaussian Prior
```cpp
class GaussianPrior : public PSDLinearSystem<ScalarType, VariableGroup<V1, V2, ...>> {

// Updates the prior error term on manifold. A * (x + dx) = b => A * x = b - A * dx; 
void update(BlockVector<ScalarType, VariableGroup<V1, V2, ...>> dx);

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


