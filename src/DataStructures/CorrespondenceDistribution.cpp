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

  // TODO use a constexpr function to compute the size this actually should be.
  enoki::set_slices(
      nearbyPotentialCorrespondences,
      MAXIMUM_CORRESPONDENCE_RADIUS * MAXIMUM_CORRESPONDENCE_RADIUS);
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
  NearbyPotentialCorrespondence<EnokiScalar*> ptr =
      enoki::slice_ptr(nearbyPotentialCorrespondences, 0);
  size_t sizeAfterCompression = 0;

  // Convert the center pixel to an enoki vector.
  enoki::Array<EnokiScalar, 2> centerPixelPacket(px_0.x(), px_0.y());

  NearbyPotentialCorrespondence<enoki::Packet<EnokiScalar>>
      nearPotentialCorrespondence;

  // Construct a nearby potential corresp. packet and compress it into the
  // array.
  auto compressionFn = [&ptr, &sizeAfterCompression, &centerPixelPacket,
                        &nearPotentialCorrespondence](auto&& pixel,
                                                      auto&& score) {
    nearPotentialCorrespondence.score = score;
    nearPotentialCorrespondence.pixelOffset = pixel - centerPixelPacket;
    nearPotentialCorrespondence.squaredDistance =
        enoki::squared_norm(nearPotentialCorrespondence.pixelOffset);

    sizeAfterCompression += enoki::compress(
        ptr, nearPotentialCorrespondence,
        nearPotentialCorrespondence.squaredDistance <=
            MAXIMUM_CORRESPONDENCE_RADIUS * MAXIMUM_CORRESPONDENCE_RADIUS);
  };
  enoki::vectorize(compressionFn, potentialCorrespondences.pixel,
                   potentialCorrespondences.score);

  enoki::set_slices(nearbyPotentialCorrespondences, sizeAfterCompression);

  // There are no close correspondences.
  if (sizeAfterCompression == 0) {
    return {};
  }

  /*
   * 2) compute the residual on the small nearby potential
   *correspondence array.
   */

  enoki::Packet<EnokiScalar> gmmPacket =
      enoki::zero<enoki::Packet<EnokiScalar>>();

  enoki::Array<enoki::Packet<EnokiScalar>, 2> weightedErrorPacket =
      enoki::zero<enoki::Array<enoki::Packet<EnokiScalar>, 2>>();

  enoki::Packet<EnokiScalar> weights;

  auto residualFn = [&gmmPacket, &weightedErrorPacket, &weights](
                        auto&& score, auto&& squaredError, auto&& pixelOffset) {
    weights = enoki::exp(squaredError / -2) * score;
    gmmPacket += weights;
    weightedErrorPacket += weights * pixelOffset;
  };
  enoki::vectorize(residualFn, nearbyPotentialCorrespondences.score,
                   nearbyPotentialCorrespondences.squaredDistance,
                   nearbyPotentialCorrespondences.pixelOffset);

  const EnokiScalar gmm = hsum(gmmPacket);
  assert(gmm > 1e-30);

  enoki::Array<EnokiScalar, 2> residual(enoki::hsum(weightedErrorPacket.x()),
                                        enoki::hsum(weightedErrorPacket.y()));
  residual /= gmm;

  assert(!std::isnan(residual.x()) && !std::isnan(residual.y()));

  return QDVO::Vector2(residual.x(), residual.y());
}

void QDVO::CorrespondenceDistribution::reset() {
  this->initialized = false;  // put this correspondence distribution to sleep.
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

  if (currentSlice > 0) {
    initialized = true;
  }

  return currentSlice;
}

Eigen::Matrix<QDVO::Scalar, 2, 2>
QDVO::CorrespondenceDistribution::fitGaussian() {
  return Sigma;
}

Eigen::Matrix<SCALAR_TYPE, Eigen::Dynamic, Eigen::Dynamic>
QDVO::CorrespondenceDistribution::extractScores(Eigen::Vector2i center,
                                                Eigen::Vector2i dimensions) {}
