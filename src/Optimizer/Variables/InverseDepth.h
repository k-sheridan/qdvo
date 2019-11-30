#pragma once

#include "Optimizer/OptimizableVariable.h"
#include <limits>

namespace ArgMin
{

/**
 * Stores the inverse depth of an observed point.
 * 
 * The inverse depth is always a member of (0, infinity].
 */
class InverseDepth : public ArgMin::OptimizableVariable<double, 1>
{
public:
    double value;

    InverseDepth() = default;

    InverseDepth(double dinv) : value(dinv) {}

    void update(const Eigen::Matrix<double, 1, 1> &dx)
    {
        // The inverse depth must come in as a valid 
        assert(value >= 0 && value <= std::numeric_limits<double>::max());
        value += dx(0,0);

        if (value < 0) {
            value = std::numeric_limits<double>::min();
        }
    }
};

} // namespace ArgMin