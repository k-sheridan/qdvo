#pragma once

#include "Types.h"

namespace QDVO {

class OptimizableVariable {

    OptimizableVariable() = default;

    virtual void update(Eigen::Matrix<QDVO::Scalar, Dimension, 1>& delta) = 0;
};

} // namespace QDVO