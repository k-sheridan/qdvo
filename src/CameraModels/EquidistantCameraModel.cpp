#include "EquidistantCameraModel.h"

QDVO::EquidistantCameraModel::EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, const Eigen::Vector4d& distortionCoeffs) :
    QDVO::CameraModel (fx, fy, cx, cy), distortionCoeffs(distortionCoeffs)
{

}
