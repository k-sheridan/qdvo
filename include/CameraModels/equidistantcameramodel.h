#ifndef EQUIDISTANTCAMERAMODEL_H
#define EQUIDISTANTCAMERAMODEL_H

#include "CameraModel.hpp"
#include "GlobalDefinitions.h"
#include <vector>

namespace QDVO {
class EquidistantCameraModel : CameraModel
{
public:
    EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, const Eigen::Vector4d& distortionCoeffs);

    Eigen::Matrix<SCALAR_TYPE, 2, 1> project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian = nullptr);

    Eigen::Matrix<SCALAR_TYPE, 3, 1> unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian = nullptr);


private:
    Eigen::Vector4d distortionCoeffs; // the 3rd, 5th, 7th, and 9th order coefficients of a polynomial function of the landmark angle. Same as used in Kalibr.

    struct UniformRadiusLookUpTable {
        std::vector<double> umap; // r = f(\theta) = umap(\theta / resolution)
        double resolution = EQUIDISTANT_CAMERA_MODEL_RADIUS_MAP_RESOLUTION; // theta = index*resolution
    } radiusLookUpTable;

};
}

#endif // EQUIDISTANTCAMERAMODEL_H
