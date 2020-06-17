#include "CorrespondenceDistribution.h"

#include <algorithm>

#include "Config.h"
#include "Landmark.h"
#include "Logging.h"
#include "Patch.h"
#include "PatchComparer.h"

QDVO::CorrespondenceDistribution::CorrespondenceDistribution() {
  Sigma.setIdentity();

  // TODO find a better size to initialize to.
  potentialCorrespondences.reserve(900);
  assert(potentialCorrespondences.capacity() == 900);

  // TODO use a constexpr function to compute the size this actually should be.
  nearbyPotentialCorrespondences.reserve(MAXIMUM_CORRESPONDENCE_RADIUS *
                                         MAXIMUM_CORRESPONDENCE_RADIUS);
}

QDVO::Result<QDVO::Vector2> QDVO::CorrespondenceDistribution::computeResidual(
    CameraModel& cameraModel, Frame& frame, const QDVO::Vector2& px_0) {
  /*
   * TODO
   * 0) Determine if it is possible to compute this residual.
   * Check bounds of initialization.
   */

  /*
   * 1) compress the potential correspondences into a nearby potential
   *correspondence array.
   */
  constexpr float maxDistance = MAXIMUM_CORRESPONDENCE_RADIUS;
  constexpr float maxDistanceSq = maxDistance * maxDistance;
  const float x0 = px_0.x();
  const float y0 = px_0.y();
  float minSquaredError = std::numeric_limits<float>::max();

  for (auto&& s : potentialCorrespondences) {
    s.dx = s.x - x0;
    s.dy = s.y - y0;
    s.squaredError = s.dx * s.dx + s.dy * s.dy;
  }

  for (const auto&& s : potentialCorrespondences) {
    if (s.squaredError < minSquaredError) {
      minSquaredError = s.squaredError;
    }
  }

  float sqErrorThreshold =
      std::min(sqrt(minSquaredError) + (float)SEARCH_RADIUS_PADDING,
               (float)MAXIMUM_CORRESPONDENCE_RADIUS);
  sqErrorThreshold = sqErrorThreshold * sqErrorThreshold;

  nearbyPotentialCorrespondences.clear();
  for (const auto&& s : potentialCorrespondences) {
    if (s.squaredError < sqErrorThreshold) {
      nearbyPotentialCorrespondences.emplace_back(s.dx, s.dy, s.score,
                                                  s.squaredError);
    }
  }

  /*
   * 2) compute the residual on the small nearby potential
   *correspondence array.
   */

  if (nearbyPotentialCorrespondences.empty()) {
    return {};
  }

  for (auto&& s : nearbyPotentialCorrespondences) {
    s.weight = std::exp(s.squaredError / -2) * s.score;
  }

  float ex = 0;
  float ey = 0;
  float gmm = 0;
  for (const auto&& s : nearbyPotentialCorrespondences) {
    ex += s.dx * s.weight;
    ey += s.dy * s.weight;
    gmm += s.weight;
  }

  return QDVO::Vector2(ex, ey) / gmm;
}

void QDVO::CorrespondenceDistribution::reset() {
  this->initialized = false;  // put this correspondence distribution to sleep.
  potentialCorrespondences.clear();
  nearbyPotentialCorrespondences.clear();
  Sigma.setIdentity();
}

int QDVO::CorrespondenceDistribution::initializeDistribution(
    CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey,
    const Eigen::Vector2i& centerPixel, const int floodRadius,
    QDVO::Patch warpedPatch, float threshold) {
  CHECK(!initialized,
        "The correspondence distribution must not be initialized.");

  this->landmarkKey = landmarkKey;
  this->warpedPatch = warpedPatch;

  const int width = floodRadius * 2 + 1;
  const int lx = std::max(0 + PATCH_RADIUS, centerPixel.x() - floodRadius);
  const int hx = std::min(cameraModel.imageWidth() - 1 - PATCH_RADIUS,
                          centerPixel.x() + floodRadius);
  const int ly = std::max(0 + PATCH_RADIUS, centerPixel.y() - floodRadius);
  const int hy = std::min(cameraModel.imageHeight() - 1 - PATCH_RADIUS,
                          centerPixel.y() + floodRadius);

  QDVO::PatchComparer comp;

  Eigen::Vector2i px;

  potentialCorrespondences.clear();

  for (int x = lx; x <= hx; ++x) {
    for (int y = ly; y <= hy; ++y) {
      px = {x, y};
      QDVO::Result<float> score = comp.compare(warpedPatch, frame, px);

      if (score.has_value() && score.value() >= threshold) {
        potentialCorrespondences.emplace_back(px.x(), px.y(), score.value());
      }
    }
  }

  if (potentialCorrespondences.size() > 0) {
    initialized = true;
  }

  return potentialCorrespondences.size();
}

Eigen::Matrix<QDVO::Scalar, 2, 2>
QDVO::CorrespondenceDistribution::fitGaussian() {
  return Sigma;
}

Eigen::Matrix<SCALAR_TYPE, Eigen::Dynamic, Eigen::Dynamic>
QDVO::CorrespondenceDistribution::extractScores(Eigen::Vector2i center,
                                                Eigen::Vector2i dimensions) {}
