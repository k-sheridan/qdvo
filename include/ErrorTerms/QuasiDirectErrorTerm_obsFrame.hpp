#ifndef QUASIDIRECTERRORTERM_OBSFRAME_HPP
#define QUASIDIRECTERRORTERM_OBSFRAME_HPP

#include <ceres/sized_cost_function.h>

namespace  QDVO {

class QuasiDirectErrorTerm_obsFrame : public ceres::SizedCostFunction<2, 6>
{
public:
    QuasiDirectErrorTerm_obsFrame();

    virtual bool Evaluate(double const* const* parameters,
                            double* residuals,
                            double** jacobians) const = 0;
};
}

#endif // QUASIDIRECTERRORTERM_OBSFRAME_HPP
