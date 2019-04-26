#include "CameraModel.hpp"

CameraModel::CameraModel()
{

}

Eigen::Matrix<SCALAR_TYPE, 2, 1> CameraModel::project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian)
{
    if (pointInCamera(2) < SMALL_NUMBER)
    {
        throw "point behind camera";
    }

    SCALAR_TYPE u = pointInCamera(0)/pointInCamera(2);
    SCALAR_TYPE v = pointInCamera(1)/pointInCamera(2);

    // optionally compute the projection jacobian
    if (projectionJacobian != nullptr)
    {
        (*projectionJacobian)(0, 0) = this->fx;
        (*projectionJacobian)(1, 1) = this->fy;
        (*projectionJacobian)(1, 0) = 0;
        (*projectionJacobian)(0, 1) = 0;
    }

    return Eigen::Matrix<SCALAR_TYPE, 2, 1>(u*this->fx + this->cx, v*this->fy + this->cy);
}

Eigen::Matrix<SCALAR_TYPE, 3, 1> CameraModel::unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian)
{
    // optionally compute the projection jacobian
    if (unprojectionJacobian != nullptr)
    {
        (*unprojectionJacobian)(0, 0) = 1/this->fx;
        (*unprojectionJacobian)(1, 1) = 1/this->fy;
        (*unprojectionJacobian)(1, 0) = 0;
        (*unprojectionJacobian)(0, 1) = 0;
    }

    return Eigen::Matrix<SCALAR_TYPE, 3, 1>((pixel(0) - this->cx)/this->fx, (pixel(1) - this->cy)/this->fy, 1);
}
