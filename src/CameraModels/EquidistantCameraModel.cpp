#include "EquidistantCameraModel.h"

QDVO::EquidistantCameraModel::EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, const Eigen::Vector4d& distortionCoeffs) :
    QDVO::CameraModel (fx, fy, cx, cy, fov), distortionCoeffs(distortionCoeffs)
{
    // generate the LUT for the radius to angle relationship.
}


Eigen::Matrix<SCALAR_TYPE, 2, 1> QDVO::EquidistantCameraModel::project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian)
{

}

Eigen::Matrix<SCALAR_TYPE, 3, 1> QDVO::EquidistantCameraModel::unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian)
{

}
