#pragma once

#include "Optimizer/OptimizableVariable.h"

namespace ArgMin
{

class InverseDepth : public ArgMin::OptimizableVariable<double, 1>
{
public:
    InverseDepth()
    {
    }

    void update(const Eigen::Matrix<double, 1, 1> &dx)
    {
    }
};

} // namespace ArgMin