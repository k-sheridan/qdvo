#ifndef QUASIDIRECTERRORTERM_OBSFRAME_HPP
#define QUASIDIRECTERRORTERM_OBSFRAME_HPP

#include <ceres/sized_cost_function.h>

namespace  QDVO {

class QuasiDirectErrorTerm_obsFrame : public ceres::SizedCostFunction<2, 3, 4>
{
public:
    QuasiDirectErrorTerm_obsFrame();

    // parameter order:
    // pos: [x, y, z], quat: [qx, qy, qz, qw]
    virtual bool Evaluate(double const* const* parameters,
                            double* residuals,
                            double** jacobians) const = 0;

private:
    // store gaussian mixture model here and potentially more.
};
}

#endif // QUASIDIRECTERRORTERM_OBSFRAME_HPP
