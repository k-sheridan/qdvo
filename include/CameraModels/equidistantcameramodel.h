#ifndef EQUIDISTANTCAMERAMODEL_H
#define EQUIDISTANTCAMERAMODEL_H

#include "CameraModel.hpp"

namespace QDVO {
class EquidistantCameraModel : CameraModel
{
public:
    EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, const Eigen::Vector4d& distortionCoeffs);

    Eigen::Vector4d distortionCoeffs; // the 3rd, 5th, 7th, and 9th order coefficients of a polynomial function of the landmark angle. Same as used in Kalibr.
};
}

#endif // EQUIDISTANTCAMERAMODEL_H
