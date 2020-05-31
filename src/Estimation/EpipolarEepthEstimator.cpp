#include "CameraModel.hpp"
#include "Config.h"
#include "DataStructures/Graph.h"
#include "DataStructures/Patch.h"
#include "EpipolarDepthEstimator.h"
#include "GlobalDefinitions.h"
#include "Logging.h"
#include "PatchComparer.h"
#include "PatchWarper.h"
#include "Types.h"

using namespace QDVO;

void EpipolarDepthEstimator::update(Graph& g, Frame& sourceKeyframe,
                                    Frame& targetKeyframe, Landmark& landmark,
                                    PatchComparer& patchComparer,
                                    PatchWarper& patchWarper) {
  // Ensure that the landmark is inactive.
  if (landmark.status != Landmark::LandmarkStatus::INACTIVE) {
    LOG_ERROR("Tried to run epipolar depth estimator on {} landmark.",
              landmark.status);
  }

  if (initialized) {
    // Return if the estimator already has finished.
    LOG_TRACE(
        "Tried to update landmark's depth estimator which was already "
        "initialized.");
    return;
  }

  // Increment the attempts.
  ++attempts;

  if (attempts > config->epipolar_depth_estimator.maximumAttempts) {
    // There have been too many attempts marginalize the landmark.
    LOG_TRACE("Too many attempts to estimate the landmark depth have occured.");
    // landmark.status = Landmark::LandmarkStatus::MARGINALIZED;
    return;
  }

  // Warp the landmark patch into the target frame.
  Result<Patch> templatePatch;
  patchWarper.warpPatchToTargetFrame(templatePatch, landmark, sourceKeyframe,
                                     targetKeyframe, g);

  // If the template patch doesnt exist, return early.
  if (!templatePatch.has_value()) {
    LOG_TRACE("Failed to warp patch for epipolar depth estimator.");
    return;
  }

  // Compute the relative transformation for this frame.
  auto T_target_source =
      g.computeRelativeKeyframeTransform(targetKeyframe, sourceKeyframe);

  // Get the target camera model.
  CameraModel& cm =
      *g.getCameraModelMap().at(targetKeyframe.cameraModelKey)->first;

  QDVO::Vector3 pointInTarget, pointInTarget_unitDepth;
  Eigen::Matrix<double, 2, 3> dPi;
  Eigen::Matrix<double, 2, 2> projJac;
  // Create a lambda which computes the pixel derivative w.r.t the landmark
  // depth. return [derivatives, pixel.]
  auto dPx_dz =
      [&](double z) -> QDVO::Result<std::pair<QDVO::Vector2, QDVO::Vector2>> {
    // TODO precompute the rotation matrix.
    pointInTarget_unitDepth =
        T_target_source.unit_quaternion() * landmark.bearing;
    pointInTarget = pointInTarget_unitDepth * z + T_target_source.translation();

    auto projectionResult = cm.project(pointInTarget, &projJac);

    if (!projectionResult.has_value()) {
      return {};
    }

    dPi << 1 / pointInTarget(2, 0), 0,
        -pointInTarget(0, 0) / (pointInTarget(2, 0) * pointInTarget(2, 0)), 0,
        1 / pointInTarget(2, 0),
        -pointInTarget(1, 0) / (pointInTarget(2, 0) * pointInTarget(2, 0));

    return std::make_pair((projJac * dPi * pointInTarget_unitDepth).eval(),
                          projectionResult.value());
  };

  // Set up the result vector.
  std::vector<double> depths, scores;
  // Search for the depth with a pixel resolution,
  double depth = config->epipolar_depth_estimator.minimumDepth;
  while (depth <= config->epipolar_depth_estimator.maximumDepth) {
    // Evaluate the pixel position and pixel derivative for this depth.
    auto pixelResult = dPx_dz(depth);

    // Extract a patch if possible
    QDVO::Result<QDVO::Patch> patchResult;
    if (pixelResult.has_value()) {
      patchResult = targetKeyframe.imagePyr.getImage().getSubPixelPatch(
          pixelResult.value().second);
    }
    // Check if there is a valid result.
    if (!patchResult.has_value()) {
      // If we failed to project after already starting the search exit.
      if (!scores.empty()) {
        break;
      } else {
        // Manually increment by a default value.
        // TODO Don't hard code this.
        depth += 0.2;
        continue;
      }
    }

    // Evaluate this pixel position.
    // If the score can be evaluated, store the depth score pair.
    auto scoreResult =
        patchComparer.compare(templatePatch.value(), patchResult.value());

    if (scoreResult.has_value()) {
      depths.push_back(depth);
      scores.push_back(scoreResult.value());
    }

    // Increment the depth using pixel derivative.
    depth += config->epipolar_depth_estimator.resolution /
             pixelResult.value().first.norm();
  }

  if (scores.empty()) {
    LOG_TRACE("Could not evaluate any depths for the current keyframe.");
    return;
  }
  // Find the maximum score.
  auto maxScoreIt = std::max_element(scores.begin(), scores.end());

  LOG_TRACE(
      "Evaluated {} depths during epipolar search. Max score: {} at depth: {}",
      depths.size(), *maxScoreIt,
      depths.at(std::distance(scores.begin(), maxScoreIt)));

  if (*maxScoreIt < POTENTIAL_CORRESPONDENCE_THRESHOLD) {
    LOG_INFO("Landmark has no match during epipolar depth search.");
    landmark.status = Landmark::LandmarkStatus::MARGINALIZED;
    return;
  }

  // Check that the scores only have a single cluster.
  int switches = 0;
  bool state = false;
  for (auto& score : scores) {
    if ((score >= POTENTIAL_CORRESPONDENCE_THRESHOLD) && state == false) {
      ++switches;
    }

    state = (score >= POTENTIAL_CORRESPONDENCE_THRESHOLD);
  }

  if (switches > 1) {
    LOG_TRACE("Landmark was not unique.");
    landmark.status = Landmark::LandmarkStatus::MARGINALIZED;
    return;
  }

  // Find the beginning and end potential correspondences.
  int startIdx = -1;
  int endIdx = -1;
  for (int idx = 0; idx < scores.size(); ++idx) {
    if (startIdx < 0 && scores.at(idx) >= POTENTIAL_CORRESPONDENCE_THRESHOLD) {
      startIdx = idx;
    }
    if (endIdx < 0 && startIdx >= 0 &&
        scores.at(idx) < POTENTIAL_CORRESPONDENCE_THRESHOLD) {
      endIdx = idx;
      break;
    }
  }

  if (endIdx < 0) {
    endIdx = scores.size() - 1;
  }

  // Compute the estimation error for this update.
  assert(startIdx >= 0);
  double thisError = depths.at(endIdx) - depths.at(startIdx);

  // If the error has decreased, update the point.
  if (error > thisError) {
    if (depths.at(std::distance(scores.begin(), maxScoreIt)) ==
        config->epipolar_depth_estimator.minimumDepth) {
      LOG_TRACE("Depth solution at lower boundary.");
      return;
    }
    error = thisError;
    landmark.dinv = 1.0 / depths.at(std::distance(scores.begin(), maxScoreIt));
    hypotheses = endIdx - startIdx + 1;
    LOG_TRACE(
        "Updated depth. New estimated depth is: {} with and error of: {} and "
        "{} hypotheses.",
        1.0 / landmark.dinv, thisError, hypotheses);
  }

  // Finally, check if the current estimate meets our initialization
  // requirements.
  if (endIdx - startIdx <= config->epipolar_depth_estimator.maximumHypotheses &&
      error <= config->epipolar_depth_estimator.maximumErrorPerDepth *
                   depths.at(std::distance(scores.begin(), maxScoreIt))) {
    LOG_TRACE(
        "Landmark depth successfully estimated with {} hypotheses and and "
        "error of {}",
        endIdx - startIdx, error);
    initialized = true;
  }
}
