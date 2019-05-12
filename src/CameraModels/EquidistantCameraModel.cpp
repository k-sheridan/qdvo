#include "EquidistantCameraModel.h"

QDVO::EquidistantCameraModel::EquidistantCameraModel(SCALAR_TYPE fx, SCALAR_TYPE fy, SCALAR_TYPE cx, SCALAR_TYPE cy, SCALAR_TYPE fov, const Eigen::Vector4d& distortionCoeffs) :
    QDVO::CameraModel (fx, fy, cx, cy, fov), distortionCoeffs(distortionCoeffs)
{
    // generate the LUT for the radius to angle relationship.
    this->radiusLookUpTable.umap.resize(std::ceil(this->fov / this->radiusLookUpTable.resolution));

    for (size_t i = 0; i < this->radiusLookUpTable.umap.size(); ++i)
    {
        // compute the associated radius for this theta
        const double theta = i * this->radiusLookUpTable.resolution;

        // do this inline for both performance and ease.
        const double theta2 = theta*theta;
        double thetaAccumulated = theta; // used to accumulate the
        this->radiusLookUpTable.umap[i] = thetaAccumulated;

        thetaAccumulated *= theta2;
        this->radiusLookUpTable.umap[i] += this->distortionCoeffs(0)*thetaAccumulated;

        thetaAccumulated *= theta2;
        this->radiusLookUpTable.umap[i] += this->distortionCoeffs(1)*thetaAccumulated;

        thetaAccumulated *= theta2;
        this->radiusLookUpTable.umap[i] += this->distortionCoeffs(2)*thetaAccumulated;

        thetaAccumulated *= theta2;
        this->radiusLookUpTable.umap[i] += this->distortionCoeffs(3)*thetaAccumulated;
    }
}


Eigen::Matrix<SCALAR_TYPE, 2, 1> QDVO::EquidistantCameraModel::project(Eigen::Matrix<SCALAR_TYPE, 3, 1> pointInCamera, Eigen::Matrix<SCALAR_TYPE, 2, 2>* projectionJacobian)
{

}

Eigen::Matrix<SCALAR_TYPE, 3, 1> QDVO::EquidistantCameraModel::unproject(Eigen::Matrix<SCALAR_TYPE, 2, 1> pixel, Eigen::Matrix<SCALAR_TYPE, 2, 2>* unprojectionJacobian)
{

}
