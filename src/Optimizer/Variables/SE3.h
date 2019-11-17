#pragma once

#include "Optimizer/OptimizableVariable.h"

namespace ArgMin
{

class SE3 : public ArgMin::OptimizableVariable<double, 6>
{
public:
    SE3()
    {
    }

    void update(const Eigen::Matrix<double, 6, 1> &dx)
    {
    }
};

} // namespace ArgMin
