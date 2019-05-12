#ifndef FOVCAMERAMODEL_H
#define FOVCAMERAMODEL_H

#include <CameraModel.hpp>

namespace  QDVO {
class FOVCameraModel : CameraModel
{
public:
    FOVCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov);
};
}

#endif // FOVCAMERAMODEL_H
