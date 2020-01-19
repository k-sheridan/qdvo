#include "EquidistantCameraModel.h"

QDVO::EquidistantCameraModel::EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, int width, int height, const Eigen::Vector4d &distortionCoeffs) : QDVO::CameraModel(fx, fy, cx, cy, fov, width, height), distortionCoeffs(distortionCoeffs)
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

QDVO::Result<QDVO::Vector2> QDVO::EquidistantCameraModel::project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2> *projectionJacobian)
{
    if (!this->isPointPotentiallyVisible(pointInCamera))
    {
        //throw std::runtime_error("point not possibly visible.");
        return {};
    }

    Eigen::Matrix<SCALAR_TYPE, 3, 1> homogenousPoint = pointInCamera / pointInCamera(2);

    SCALAR_TYPE normPointProjectedOntoImagePlane = sqrt(pointInCamera(0) * pointInCamera(0) + pointInCamera(1) * pointInCamera(1));
    SCALAR_TYPE theta = atan2(normPointProjectedOntoImagePlane, std::abs(pointInCamera(2)));

    if (theta > this->fov / 2.0)
    {
        //throw std::runtime_error("point out of the field of view");
        return {};
    }

    SCALAR_TYPE psi = atan2(pointInCamera(1), pointInCamera(0));

    SCALAR_TYPE radius = this->distortionFn(theta);

    Eigen::Matrix<SCALAR_TYPE, 2, 1> bearingDistorted(radius * cos(psi), radius * sin(psi));

    Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel(bearingDistorted(0) * this->fx + this->cx, bearingDistorted(1) * this->fy + this->cy);

    if (!this->isPixelOnImage(pixel))
    {
        //throw std::runtime_error("pixel not on image");
        return {};
    }

    // check if the projection jacobian should be computed
    if (projectionJacobian != nullptr)
    {
        try
        {
            const SCALAR_TYPE delta = 1e-4;

            projectionJacobian->block(0, 0, 2, 1) = this->project(homogenousPoint.block(0, 0, 3, 1) + Eigen::Matrix<SCALAR_TYPE, 3, 1>(delta, 0, 0)).value() - this->project(homogenousPoint.block(0, 0, 3, 1) - Eigen::Matrix<SCALAR_TYPE, 3, 1>(delta, 0, 0)).value();

            projectionJacobian->block(0, 1, 2, 1) = this->project(homogenousPoint.block(0, 0, 3, 1) + Eigen::Matrix<SCALAR_TYPE, 3, 1>(0, delta, 0)).value() - this->project(homogenousPoint.block(0, 0, 3, 1) - Eigen::Matrix<SCALAR_TYPE, 3, 1>(0, delta, 0)).value();

            *(projectionJacobian) /= (2 * delta);
        }
        catch (std::bad_optional_access &e)
        {
            std::cout << "failed to numerically evaluate the projection jacobian" << std::endl;
            return {};
        }
    }

    return pixel;
}

QDVO::Result<QDVO::Vector3> QDVO::EquidistantCameraModel::unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2> *unprojectionJacobian)
{
    Eigen::Matrix<SCALAR_TYPE, 2, 1> distortedBearing((pixel(0) - this->cx) / this->fx, (pixel(1) - this->cy) / this->fy);

    SCALAR_TYPE psi = atan2(distortedBearing(1), distortedBearing(0));

    SCALAR_TYPE desiredRadius = (distortedBearing).norm();

    size_t highIdx = this->radiusLookUpTable.umap.size() - 1;
    size_t lowIdx = 0;
    size_t midIdx = highIdx / 2;

    while (true)
    {
        SCALAR_TYPE rEval = this->radiusLookUpTable.umap.at(midIdx);

        if (highIdx - 1 <= lowIdx)
        {
            break;
        } // converged

        if (desiredRadius > rEval)
        {
            lowIdx = midIdx;
            midIdx = lowIdx + (highIdx - lowIdx) / 2;
        }
        else if (desiredRadius < rEval)
        {
            highIdx = midIdx;
            midIdx = lowIdx + (highIdx - lowIdx) / 2;
        }
        else
        {
            break;
        }
    }

    SCALAR_TYPE theta0 = midIdx * this->radiusLookUpTable.resolution;

    if (std::abs(theta0) > this->fov / 2.0)
    {
        //throw std::runtime_error("unproject in unstable region!");
        return {};
    }

    SCALAR_TYPE theta = theta0;
    for (int i = 0; i < 20; ++i) // should converge way before 20 iters depending on table resolution
    {
        SCALAR_TYPE der = this->distortionFnDerivative(theta);

        if (std::abs(der) <= 1e-8)
        {
            break;
        }

        theta = theta + (desiredRadius - this->distortionFn(theta)) / der;
    }

    // now the undistorted radius has been determined.
    SCALAR_TYPE tTh = tan(theta);
    Eigen::Matrix<SCALAR_TYPE, 3, 1> bearing(tTh * cos(psi), tTh * sin(psi), 1);

    if (unprojectionJacobian != nullptr)
    {
        Eigen::Matrix<SCALAR_TYPE, 2, 2> projJac;
        this->project(bearing, &projJac);
        *(unprojectionJacobian) = projJac.inverse();
    }

    return bearing;
}
