#include "FOVCameraModel.hpp"

QDVO::FOVCameraModel::FOVCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height) :
    QDVO::CameraModel (fx, fy, cx, cy, fov, width, height)
{

}
