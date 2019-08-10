#include "CameraModel.hpp"

QDVO::CameraModel::CameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height) :
    cx(cx), cy(cy), fx(fx), fy(fy), fov(fov), width(width), height(height)
{

}

QDVO::Result<QDVO::Vector2> QDVO::CameraModel::project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian)
{
    if (pointInCamera(2) < SMALL_NUMBER)
    {
        return {};
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

    return QDVO::Vector2(u*this->fx + this->cx, v*this->fy + this->cy);
}

QDVO::Result<QDVO::Vector3> QDVO::CameraModel::unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian)
{
    // optionally compute the unprojection jacobian
    if (unprojectionJacobian != nullptr)
    {
        (*unprojectionJacobian)(0, 0) = 1/this->fx;
        (*unprojectionJacobian)(1, 1) = 1/this->fy;
        (*unprojectionJacobian)(1, 0) = 0;
        (*unprojectionJacobian)(0, 1) = 0;
    }

    return Eigen::Matrix<SCALAR_TYPE, 3, 1>((pixel(0) - this->cx)/this->fx, (pixel(1) - this->cy)/this->fy, 1);
}

bool QDVO::CameraModel::isPointPotentiallyVisible(const Eigen::Matrix<SCALAR_TYPE, 3, 1>& pointInCamera)
{
    if (pointInCamera(2) < SMALL_NUMBER){
        return false;
    }

    return true;
}

bool QDVO::CameraModel::isPixelOnImage(const Eigen::Matrix<double, 2, 1> &pixel)
{
    if (pixel(0) >= 0 && pixel(0) < this->width - 1 && pixel(1) >= 0 && pixel(1) < this->height - 1)
    {
        return true;
    }
    else
    {
        return false;
    }

}
