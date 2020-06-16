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
  // TODO find a better size to initialize to.
  enoki::set_slices(potentialCorrespondences, 900);
  assert(potentialCorrespondences.score.capacity() == 900);
  assert(potentialCorrespondences.pixel[0].capacity() == 900);
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

  const int width = floodRadius * 2 + 1;
  const int lx = std::max(0 + PATCH_RADIUS, centerPixel.x() - floodRadius);
  const int hx = std::min(cameraModel.imageWidth() - 1 - PATCH_RADIUS,
                          centerPixel.x() + floodRadius);
  const int ly = std::max(0 + PATCH_RADIUS, centerPixel.y() - floodRadius);
  const int hy = std::min(cameraModel.imageHeight() - 1 - PATCH_RADIUS,
                          centerPixel.y() + floodRadius);

  QDVO::PatchComparer comp;

  Eigen::Vector2i px;
  int currentSlice = 0;

  for (int x = lx; x <= hx; ++x) {
    for (int y = ly; y <= hy; ++y) {
      px = {x, y};
      QDVO::Result<float> score = comp.compare(warpedPatch, frame, px);

      if (score.has_value() && score.value() >= threshold) {
        if (currentSlice >= potentialCorrespondences.score.capacity()) {
          auto temp = potentialCorrespondences;
          enoki::set_slices(potentialCorrespondences, (currentSlice + 1) * 2);
          potentialCorrespondences = temp;
          enoki::set_slices(potentialCorrespondences, (currentSlice + 1) * 2);
          assert(currentSlice < potentialCorrespondences.score.capacity());
        }

        auto&& s = enoki::slice(potentialCorrespondences, currentSlice);

        s.score = score.value();
        s.pixel[0] = px.x();
        s.pixel[1] = px.y();

        ++currentSlice;
      }
    }
  }

  enoki::set_slices(potentialCorrespondences, currentSlice);
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
