#include "EquidistantCameraModel.h"


QDVO::EquidistantCameraModel::EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height, const Eigen::Vector4d& distortionCoeffs) :
    QDVO::CameraModel (fx, fy, cx, cy, fov, width, height), distortionCoeffs(distortionCoeffs)
{
    // generate the LUT for the radius to angle relationship.
    this->radiusLookUpTable.umap.resize(std::ceil(this->fov / this->radiusLookUpTable.resolution));

    for (size_t i = 0; i < this->radiusLookUpTable.umap.size(); ++i)
    {
        // compute the associated radius for this theta
        const double theta = i * this->radiusLookUpTable.resolution;

        this->radiusLookUpTable.umap[i] = this->distortionFn(theta);
    }
}



Eigen::Matrix<SCALAR_TYPE, 2, 1> QDVO::EquidistantCameraModel::project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian)
{
    if (!this->isPointPotentiallyVisible(pointInCamera))
    {
        throw std::runtime_error("point not possibly visible.");
    }

    Eigen::Matrix<SCALAR_TYPE, 3, 1> homogenousPoint = pointInCamera / pointInCamera(2);

    SCALAR_TYPE normPointProjectedOntoImagePlane = sqrt(pointInCamera(0)*pointInCamera(0) + pointInCamera(1)*pointInCamera(1));
    SCALAR_TYPE theta = atan2(normPointProjectedOntoImagePlane, abs(pointInCamera(2)));

    if (theta > this->fov / 2.0)
    {
        throw std::runtime_error("point out of the field of view");
    }

    SCALAR_TYPE psi = atan2(pointInCamera(1), pointInCamera(0));

    SCALAR_TYPE radius = this->distortionFn(theta);

    Eigen::Matrix<SCALAR_TYPE, 2, 1> bearingDistorted(radius * cos(psi), radius * sin(psi));

    Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel(bearingDistorted(0) * this->fx + this->cx, bearingDistorted(1) * this->fy + this->cy);

    if (!this->isPixelOnImage(pixel))
    {
        throw std::runtime_error("pixel not on image");
    }

    // check if the projection jacobian should be computed
    if (projectionJacobian != nullptr)
    {
        const SCALAR_TYPE delta = 1e-4;

        projectionJacobian->block(0, 0, 2, 1) = this->project(homogenousPoint.block(0, 0, 3, 1) + Eigen::Matrix<SCALAR_TYPE, 3, 1>(delta, 0, 0))
                - this->project(homogenousPoint.block(0, 0, 3, 1) - Eigen::Matrix<SCALAR_TYPE, 3, 1>(delta, 0, 0));

        projectionJacobian->block(0, 1, 2, 1) = this->project(homogenousPoint.block(0, 0, 3, 1) + Eigen::Matrix<SCALAR_TYPE, 3, 1>(0, delta, 0))
                - this->project(homogenousPoint.block(0, 0, 3, 1) - Eigen::Matrix<SCALAR_TYPE, 3, 1>(0, delta, 0));

        *(projectionJacobian) /= (2*delta);
    }

    return pixel;
}

Eigen::Matrix<SCALAR_TYPE, 3, 1> QDVO::EquidistantCameraModel::unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian)
{

}

