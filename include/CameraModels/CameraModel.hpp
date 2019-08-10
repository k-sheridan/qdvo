#ifndef CAMERAMODEL_HPP
#define CAMERAMODEL_HPP

#include <Eigen/Core>
#include "GlobalDefinitions.h"
#include "Types.h"

/*
 * Base Class for a generic camera model. This base class is implemented as a pinhole camera.
 * There are two main functions:
 *  project: convert metric camera point to pixel
 *  unproject: convert pixel to metric camera point on the unit plane.
 *
 * The constructor can be used to both preallocate and compute mapping used to speed up the project/unproject functions.
 */

namespace  QDVO {
class CameraModel
{
public:

    CameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height);

    virtual QDVO::Result<QDVO::Vector2> project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian = nullptr);

    virtual QDVO::Result<QDVO::Vector3> unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian = nullptr);

    virtual bool isPointPotentiallyVisible(const QDVO::Vector3& pointInCamera);

    virtual bool isPixelOnImage(const QDVO::Vector2& pixel);

    SCALAR_TYPE fx, fy, cx, cy, fov; // intrinsic parameters.
    int width, height;

};
}

#endif // CAMERAMODEL_HPP
