#ifndef HACKYLOCALPARAMETERIZATIONSO3_H
#define HACKYLOCALPARAMETERIZATIONSO3_H

#include <sophus/so3.hpp>
#include "GlobalDefinitions.h"
#include <algorithm>
#include <iterator>
#include <ceres/local_parameterization.h>

namespace QDVO {

class HackyLocalParameterizationSO3 : public ceres::LocalParameterization
{
public:
    virtual ~HackyLocalParameterizationSO3() {}


    /**
     * A generalized plus operation for SO(3)
     */
    virtual bool Plus(double const* T_raw, double const* delta_raw,
                      double* T_plus_delta_raw) const {
        Eigen::Map<Sophus::SO3d const> const T(T_raw);
        Eigen::Map<Sophus::Vector3d const> const delta(delta_raw);
        Eigen::Map<Sophus::SO3d> T_plus_delta(T_plus_delta_raw);
        T_plus_delta = T * Sophus::SO3d::exp(delta);
        return true;
    }

    /**
     * computes the jacobian which maps the global matrix to local matrix through:
     * local = global * J
     * This is a hacky way of allowing us to compute locally paramterized jacobians in the error terms.
     */
    virtual bool ComputeJacobian(double const* T_raw,
                                 double* jacobian_raw) const
    {
        static Eigen::Matrix<double, 7, 6, Eigen::RowMajor> eye(Eigen::Matrix<double, 7, 6, Eigen::RowMajor>::Identity());

        jacobian_raw = eye.array().data();

        return true;
    }

    /**
     * actually computes the local matrix from the global matrix.
     * For this operation, it is assumed that the global matrix is actually the local matrix with an extra column.
     */
    virtual bool MultiplyByJacobian(const double *x, const int num_rows, const double *global_matrix, double *local_matrix) const
    {
        for(int i = 0; i < num_rows; ++i)
        {
            for(int j = 0; j < Sophus::SO3<double>::DoF; ++j)
            {
                *(local_matrix + i*Sophus::SO3<double>::num_parameters + j) = *(global_matrix + i*Sophus::SO3<double>::num_parameters + j);
            }
        }
        return true;
    }

    virtual int GlobalSize() const { return Sophus::SO3<double>::num_parameters; }

    virtual int LocalSize() const { return Sophus::SO3<double>::DoF; }
};
}

#endif // LOCALPARAMETERIZATIONSO3_H
