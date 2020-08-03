#include "CorrespondenceDistribution.h"

#include <algorithm>

#include "Config.h"
#include "Landmark.h"
#include "Logging.h"
#include "Patch.h"
#include "PatchComparer.h"

SCALAR_TYPE computeWidthRatio(QDVO::Frame& frame, int fromLevel, int toLevel) {
  return ((SCALAR_TYPE)(frame.imagePyr.getImage(toLevel).cols())) /
         ((SCALAR_TYPE)(frame.imagePyr.getImage(fromLevel).cols()));
}

SCALAR_TYPE computeHeightRatio(QDVO::Frame& frame, int fromLevel, int toLevel) {
  return ((SCALAR_TYPE)(frame.imagePyr.getImage(toLevel).rows())) /
         ((SCALAR_TYPE)(frame.imagePyr.getImage(fromLevel).rows()));
}

QDVO::CorrespondenceDistribution::CorrespondenceDistribution(
    unsigned width, unsigned height,
    std::shared_ptr<const RadialSearchPattern> searchPattern)
    : width(width), height(height) {
  this->correspondenceMap =
      QDVO::SpatialMap<PotentialCorrespondence>(std::max(width, height));
  this->radialSearchPattern = std::move(searchPattern);
  Sigma.setIdentity();
  widthRatio = 1;
  heightRatio = 1;
}

QDVO::Result<QDVO::Vector2> QDVO::CorrespondenceDistribution::computeResidual(
    CameraModel& cameraModel, Frame& frame, const QDVO::Vector2& px_0) {
  CHECK(initialized, "The correspondence distribution must be initialized.");

  // Look for the closest potential correspondences approximately under a
  // certain radius using the generic quadtree. While looking compute the
  // gaussians
  this->search(cameraModel, frame, px_0, MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS,
               true, errorArray, scoreArray);

  // Compute exp(error.norm()/2) * score
  expScoreArray.resize(scoreArray.size());
  std::transform(scoreArray.begin(), scoreArray.end(), errorArray.begin(),
                 expScoreArray.begin(),
                 [](SCALAR_TYPE score, QDVO::Vector2 error) -> SCALAR_TYPE {
                   return exp(-0.5 * error.squaredNorm()) * score;
                 });

  // Compute the gmm at the center pixel.
  SCALAR_TYPE gmm = std::accumulate(
      expScoreArray.begin(), expScoreArray.begin() + scoreArray.size(), 0.0);

  if (gmm < std::numeric_limits<SCALAR_TYPE>::min()) {
    // This should never happen, but it could.
    LOG_TRACE(
        "Gaussian mixture evaluated to a small value {} with {} potential "
        "correspondences.",
        gmm, scoreArray.size());
    return {};
  }

  weightedErrorArray.resize(scoreArray.size());

  // Compute weighted errors.
  std::transform(
      errorArray.begin(), errorArray.begin() + scoreArray.size(),
      expScoreArray.begin(), weightedErrorArray.begin(),
      [gmm](QDVO::Vector2 error, SCALAR_TYPE expScore) -> QDVO::Vector2 {
        return (expScore / gmm) * error;
      });

  // Compute the residual on the patch level.
  auto residualScaled = std::accumulate(
      weightedErrorArray.begin(),
      weightedErrorArray.begin() + scoreArray.size(), QDVO::Vector2(0, 0));

  // Scale the residual back to level 0.
  return QDVO::Vector2(
      (computeWidthRatio(frame, warpedPatch.getLevel(), 0) * residualScaled(0)),
      (computeHeightRatio(frame, warpedPatch.getLevel(), 0) *
       residualScaled(1)));
}

void QDVO::CorrespondenceDistribution::reset() {
  this->patchComparer =
      nullptr;  // ensure that we cannot use the wrong patch comparison
  this->initialized = false;  // put this correspondence distribution to sleep.
  this->correspondenceMap
      .reset();  // wipe the actual distribution container clean.
}

int QDVO::CorrespondenceDistribution::initializeDistribution(
    CameraModel& cameraModel, Frame& frame, LandmarkMap::key_type landmarkKey,
    const Eigen::Vector2i& centerPixel, const int floodRadius,
    std::shared_ptr<QDVO::PatchComparer> patchComparer, QDVO::Patch warpedPatch,
    QDVO::Scalar correspondenceThreshold) {
  CHECK(!initialized,
        "The correspondence distribution must not be initialized.");

  // Set the correspondence threshold;
  potentialCorrespondenceThreshold = correspondenceThreshold;

  this->warpedPatch = warpedPatch;  // replace the patch

  this->patchComparer = std::move(
      patchComparer);  // set a new patch comparer for this distribution

  this->landmarkKey = landmarkKey;

  // ratio from original to patch level.
  widthRatio = computeWidthRatio(frame, 0, warpedPatch.getLevel());
  heightRatio = computeHeightRatio(frame, 0, warpedPatch.getLevel());

  // perform the search
  this->search(cameraModel, frame, centerPixel.cast<SCALAR_TYPE>(), floodRadius,
               false, errorArray, scoreArray);

  LOG_TRACE("initialized distribution with {} correspondences {}, {}",
            scoreArray.size(), centerPixel[0], centerPixel[1]);

  // If there is at least on potential correspondence mark this dist as
  // initilaized.
  if (!scoreArray.empty()) {
    initialized = true;
    // Fit a gaussian.
    if (config->fitGaussian) {
      Sigma = fitGaussian(frame, errorArray, scoreArray);
    } else {
      Sigma.setIdentity();
      Sigma(0, 0) = 1.0 / (widthRatio * widthRatio);
      Sigma(1, 1) = 1.0 / (heightRatio * heightRatio);
    }
  } else {
    LOG_TRACE("Could not initialize the correspondence distribution.");
  }

  return scoreArray.size();
}

Eigen::Matrix<QDVO::Scalar, 2, 2> QDVO::CorrespondenceDistribution::fitGaussian(
    QDVO::Frame& frame, std::vector<QDVO::Vector2>& errors,
    std::vector<QDVO::Scalar>& scores) {
  CHECK(errors.size() == scores.size(), "Vectors have different size.");

  Eigen::Matrix<QDVO::Scalar, 2, 2> Sigma;
  Sigma.setIdentity();

  if (scores.empty()) {
    LOG_DEBUG("Failed to compute gaussian fit.");
    return Sigma;
  }

  auto maxScoreIt = std::max_element(scores.begin(), scores.end());
  int index = std::distance(scores.begin(), maxScoreIt);

  auto centerPos = errors.at(index);

  QDVO::Vector2 temp;
  for (int i = 0; i < scores.size(); ++i) {
    temp = (errors.at(i) - centerPos);
    Sigma += scores.at(i) * temp * temp.transpose();
  }
  // LOG_INFO("Sig: {},{},{},{}", Sigma(0), Sigma(1), Sigma(2), Sigma(3));

  Eigen::Matrix<double, 2, 2> scaleMatrix;
  scaleMatrix << computeWidthRatio(frame, warpedPatch.getLevel(), 0), 0, 0,
      computeHeightRatio(frame, warpedPatch.getLevel(), 0);

  return Sigma;
}

void QDVO::CorrespondenceDistribution::search(
    CameraModel& cameraModel, Frame& frame,
    const QDVO::Vector2& centerPixelScalar, const unsigned searchRadius,
    bool minimalSearch, std::vector<QDVO::Vector2>& errors,
    std::vector<SCALAR_TYPE>& scores) {
  // Clear the error vector.
  errors.clear();
  // Clear the score vector.
  scores.clear();

  // Scaled the center pixel to the patch level.
  QDVO::Vector2 centerPixelScaled(
      (computeWidthRatio(frame, 0, warpedPatch.getLevel()) *
       centerPixelScalar(0)),
      (computeHeightRatio(frame, 0, warpedPatch.getLevel()) *
       centerPixelScalar(1)));
  // Compute the integer center pixel.
  Eigen::Vector2i centerPixel(std::round(centerPixelScaled(0)),
                              std::round(centerPixelScaled(1)));

  // Start the search.
  bool firstInfluentialPotentialCorrespondenceFound = false;
  int radiusCounter = SEARCH_RADIUS_PADDING;
  assert(this->patchComparer != nullptr);

  // start the radial search
  for (unsigned r = 0; r <= searchRadius;
       ++r)  // starting at radius 0 going to radius max.
  {
    // Is this the last radius to search?
    if (minimalSearch && firstInfluentialPotentialCorrespondenceFound) {
      if (radiusCounter-- <= 0) {
        break;
      }
    }

    for (auto& delta : this->radialSearchPattern->searchPattern.at(r)) {
      Eigen::Vector2i testPoint =
          centerPixel + delta;  // this is a point on a constant radius.

      // Make sure that this testPoint Is On The Image.
      if (!cameraModel.isPixelOnImage(testPoint)) {
        continue;
      }

      QDVO::CorrespondenceDistribution::PotentialCorrespondence* pc =
          &(this->correspondenceMap.get(testPoint));

      // Only run the patch comparison on uninitialized potential
      // correspondences.
      if (!pc->initialized()) {
        // try to compare the patch at the testPoint.
        auto score =
            this->patchComparer->compare(this->warpedPatch, frame, testPoint);

        // Add the score to the distribution
        if (score.has_value()) {
          pc->score = score.value();
        } else {
          // Set this to a bad match.
          pc->score = 0;
        }
      }

      // If the potential correspondence is good enough, it is influential.
      if (pc->score > potentialCorrespondenceThreshold) {
        firstInfluentialPotentialCorrespondenceFound = true;
        // Compute z - px0
        errors.push_back(testPoint.cast<SCALAR_TYPE>() - centerPixelScaled);
        // Add the score.
        scores.push_back((pc->score - potentialCorrespondenceThreshold) /
                         (1 - potentialCorrespondenceThreshold));
      }
    }
  }
}

Eigen::Matrix<SCALAR_TYPE, Eigen::Dynamic, Eigen::Dynamic>
QDVO::CorrespondenceDistribution::extractScores(Eigen::Vector2i center,
                                                Eigen::Vector2i dimensions) {
  // Ensure that the dimensions are odd.
  CHECK(dimensions.x() % 2 == 1, "");
  CHECK(dimensions.y() % 2 == 1, "");

  Eigen::Matrix<SCALAR_TYPE, Eigen::Dynamic, Eigen::Dynamic> result(
      dimensions.y(), dimensions.x());
  result.setConstant(-1);

  // Iterate through the region.
  for (int x = -(dimensions.x() - 1) / 2; x <= (dimensions.x() - 1) / 2; ++x) {
    for (int y = -(dimensions.y() - 1) / 2; y <= (dimensions.y() - 1) / 2;
         ++y) {
      auto px = center + Eigen::Vector2i(x, y);
      double score = -1;
      if (px(0) >= 0 && px(1) >= 0 && px(0) < width && px(1) < height) {
        auto& pc = correspondenceMap.get(Eigen::Vector2i(
            std::round(widthRatio * px(0)), std::round(heightRatio * px(1))));
        if (pc.initialized()) {
          score = pc.score;
        }
      }

      result(y + (dimensions.y() - 1) / 2, x + (dimensions.x() - 1) / 2) =
          score;
    }
  }

  return result;
}
