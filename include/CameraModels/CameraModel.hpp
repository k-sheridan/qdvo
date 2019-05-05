#ifndef CAMERAMODEL_HPP
#define CAMERAMODEL_HPP

#include <Eigen/Core>
#include <GlobalDefinitions.h>

/*
 * Base Class for a generic camera model. This base class is implemented as a pinhole camera.
 * There are two main functions:
 *  project: convert metric camera point to pixel
 *  unproject: convert pixel to metric camera point on the unit plane.
 *
 * The constructor can be used to both preallocate and compute mapping used to speed up the project/unproject functions.
 */

class CameraModel
{
public:

    SCALAR_TYPE fx, fy, cx, cy; // intrinsic parameters.

    CameraModel();

    Eigen::Matrix<SCALAR_TYPE, 2, 1> project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian);

    Eigen::Matrix<SCALAR_TYPE, 3, 1> unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian);

};

#endif // CAMERAMODEL_HPP
