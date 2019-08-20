#ifndef FOVCAMERAMODEL_H
#define FOVCAMERAMODEL_H

#include "CameraModel.hpp"

namespace  QDVO {
class FOVCameraModel : public CameraModel
{
public:
    FOVCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height);
};
}

#endif // FOVCAMERAMODEL_H
