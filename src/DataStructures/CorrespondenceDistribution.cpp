#include "CorrespondenceDistribution.h"

#include <enoki/array.h>
#include <enoki/dynamic.h>

#include <algorithm>

#include "Config.h"
#include "Landmark.h"
#include "Logging.h"
#include "Patch.h"
#include "PatchComparer.h"

QDVO::CorrespondenceDistribution::CorrespondenceDistribution() {
  Sigma.setIdentity();
}

QDVO::Result<QDVO::Vector2> QDVO::CorrespondenceDistribution::computeResidual(
    CameraModel& cameraModel, Frame& frame, const QDVO::Vector2& px_0) {}

void QDVO::CorrespondenceDistribution::reset() {
  this->initialized = false;  // put this correspondence distribution to sleep.
}

int QDVO::CorrespondenceDistribution::initializeDistribution(
    CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey,
    const Eigen::Vector2i& centerPixel, const int floodRadius,
    QDVO::Patch warpedPatch, float threshold) {
  CHECK(!initialized,
        "The correspondence distribution must not be initialized.");
}

Eigen::Matrix<QDVO::Scalar, 2, 2>
QDVO::CorrespondenceDistribution::fitGaussian() {}

void QDVO::CorrespondenceDistribution::search(
    CameraModel& cameraModel, Frame& frame,
    const QDVO::Vector2& centerPixelScalar, const unsigned searchRadius,
    bool minimalSearch) {}

Eigen::Matrix<SCALAR_TYPE, Eigen::Dynamic, Eigen::Dynamic>
QDVO::CorrespondenceDistribution::extractScores(Eigen::Vector2i center,
                                                Eigen::Vector2i dimensions) {}
