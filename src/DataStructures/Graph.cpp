#include "Graph.h"

#include "CameraModel.hpp"
#include "Frame.h"
#include "Landmark.h"
#include "Logging.h"

namespace QDVO {

Graph::Graph() {
  currentFrameKey = keyframes.insert(std::make_unique<QDVO::Frame>());
  LOG_INFO("Initialized current frame.");
  bufferFrameKey = keyframes.insert(std::make_unique<QDVO::Frame>());
  previousFrameKey = bufferFrameKey;
  LOG_INFO("Initialized previous frame.");
}

KeyframeMap::key_type Graph::findLatestFrameKey() {
  KeyframeMap::key_type result;
  for (auto it = keyframes.begin(); it != keyframes.end(); it++) {
    auto key = keyframes.getKeyFromIterator(it);
    // Only initialized keyframes are returned.
    if (!(*it)->initialized) {
      continue;
    }

    if (result.isInvalid()) {
      result = key;
      continue;
    }

    if ((*it)->imustate.time > (*keyframes.at(result))->imustate.time) {
      result = key;
      continue;
    }
  }
  return result;
}

QDVO::Frame& Graph::findLatestFrame() {
  auto key = findLatestFrameKey();
  auto frameIt = keyframes.at(key);
  CHECK(frameIt != keyframes.end(), "latest frame does not exist.");
  return *(*frameIt);
}

void Graph::swapCurrentAndBufferFrame() {
  std::swap(currentFrameKey, bufferFrameKey);

  // Find the latest keyframe key and set the new previous keyframeId
  auto latestFrameKey = findLatestFrameKey();
  if (!latestFrameKey.isInvalid()) {
    LOG_INFO("Updated previous frame key");
    previousFrameKey = latestFrameKey;
  }
}

void Graph::moveCurrentFrameIntoNewKeyframePosition() {
  // make room for another keyframe
  auto newFrame = std::make_unique<Frame>();
  auto newKeyframeKey = keyframes.insert(std::move(newFrame));

  // copy over vital information
  std::unique_ptr<Frame>& newKeyframe = *(keyframes.at(newKeyframeKey));
  newKeyframe->imustate = getCurrentFrame()->imustate;
  newKeyframe->cameraModelKey = getCurrentFrame()->cameraModelKey;
  newKeyframe->extrinsicKey = getCurrentFrame()->extrinsicKey;

  // swap the current frame into its new spot.
  currentFrameKey = newKeyframeKey;
}

void Graph::moveCurrentFrameIntoKeyframePosition() {
  // there is enough room to make a new keyframe.
  this->moveCurrentFrameIntoNewKeyframePosition();
}

void Graph::removeMarginalizedVariables() {
  // Create a list of marginalized keys.
  std::vector<LandmarkMap::key_type> landmarkKeysToRemove;
  for (auto it = landmarks.begin(); it != landmarks.end(); it++) {
    if (it->status == Landmark::LandmarkStatus::MARGINALIZED) {
      auto lKey = landmarks.getKeyFromIterator(it);
      // Only remove landmarks associated to
      auto keyframeIt = keyframes.at(it->parentFrameKey);
      if (keyframeIt != keyframes.end() &&
          (*keyframeIt)->status != QDVO::Frame::FrameStatus::MARGINALIZED) {
        // If the parent frame still exists skip this landmark.
        continue;
      }
      landmarkKeysToRemove.push_back(lKey);
    }
  }

  // erase the marginalized keys.
  LOG_TRACE("Removing {} landmarks", landmarkKeysToRemove.size());
  for (auto lKey : landmarkKeysToRemove) {
    landmarks.erase(lKey);
  }

  // Create a list of marginalized keys.
  std::vector<KeyframeMap::key_type> keyframeKeysToRemove;
  for (auto it = keyframes.begin(); it != keyframes.end(); it++) {
    if ((*it)->status == Frame::FrameStatus::MARGINALIZED) {
      keyframeKeysToRemove.push_back(keyframes.getKeyFromIterator(it));
    }
  }

  // erase the marginalized keys.
  LOG_TRACE("Removing {} keyframes", keyframeKeysToRemove.size());
  for (auto lKey : keyframeKeysToRemove) {
    keyframes.erase(lKey);
  }

  // Clean up the keyframe landmark keys.
  for (auto& keyframePtr : keyframes) {
    auto& keyframe = *keyframePtr;
    for (auto it = keyframe.landmarkKeys.begin();
         it != keyframe.landmarkKeys.end();) {
      auto landmarkIt = landmarks.at(*it);
      if (landmarkIt == landmarks.end()) {
        it = keyframe.landmarkKeys.erase(it);
      } else {
        ++it;
      }
    }
  }
}

std::vector<std::tuple<LandmarkMap::key_type, Vector2>>
Graph::getVisibleLandmarksInCurrentFrame(bool activeLandmarksOnly,
                                         bool includeCurrentFrameLandmarks) {
  LOG_INFO("Computing visible landmarks in current frame.");
  std::vector<std::tuple<LandmarkMap::key_type, Vector2>> visibleLandmarkPtrs;

  QDVO::Frame& currentFrame = *(*keyframes.at(findLatestFrameKey()));

  auto& cm = cameraModelMap.at(currentFrame.cameraModelKey)->first;

  const SE3& T_cfimu_cfcam = *(extrinsics.at(currentFrame.extrinsicKey));

  // iterate through all keyframes and project their landmarks into the current
  // frame
  for (auto& keyframe : keyframes) {
    if (keyframe->status != QDVO::Frame::FrameStatus::ACTIVE) {
      LOG_INFO("skipping inactive or marginalized keyframe.");
      continue;
    }

    LOG_INFO("computing visible landmarks for current frame.");

    const SE3& T_kfimu_kfcam = *(extrinsics.at(keyframe->extrinsicKey));

    // inv(T_w_cimu * T_imu_cam) * T_w_kimu * T_imu_cam
    SE3 T_cf_kf = (currentFrame.imustate.getSE3() * T_cfimu_cfcam).inverse() *
                  (keyframe->imustate.getSE3() * T_kfimu_kfcam);

    for (auto& lKey : keyframe->landmarkKeys) {
      // Get the landmark.
      auto landmarkIt = landmarks.at(lKey);
      if (landmarkIt == landmarks.end()) {
        LOG_TRACE("keyframe contained invalid landmark key");
        continue;
      }
      auto& l = *(landmarkIt);

      if ((l.status == Landmark::LandmarkStatus::ACTIVE ||
           !activeLandmarksOnly) &&
          l.status != Landmark::LandmarkStatus::MARGINALIZED) {
        // project landmarks
        auto px = cm->project(T_cf_kf * l.getEuclideanPoint());
        if (!px.has_value()) {
          LOG_TRACE("failed to project landmark");
          continue;
        }

        // add to vector

        visibleLandmarkPtrs.push_back(std::make_tuple(lKey, px.value()));
      }
    }
  }

  if (includeCurrentFrameLandmarks) {
    for (auto& lKey : currentFrame.landmarkKeys) {
      // Get the landmark.
      auto& l = *(landmarks.at(lKey));

      if ((l.status == Landmark::LandmarkStatus::ACTIVE ||
           !activeLandmarksOnly) &&
          l.status != Landmark::LandmarkStatus::MARGINALIZED) {
        visibleLandmarkPtrs.push_back(std::make_tuple(lKey, l.px));
      }
    }
  }

  return visibleLandmarkPtrs;
}

Vector3 Graph::projectLandmarkToCameraFrame(
    KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey,
    LandmarkMap::key_type landmarkKey) {
  std::unique_ptr<Frame>& targetFrame = *(keyframes.at(targetFrameKey));
  std::unique_ptr<Frame>& sourceFrame = *(keyframes.at(sourceFrameKey));

  auto landmarkIt = landmarks.at(landmarkKey);
  CHECK(landmarkIt != landmarks.end(), "Landmark does not exist.");
  Landmark& l = *(landmarkIt);

  CHECK(l.parentFrameKey == sourceFrameKey,
        "Landmark is not child of source keyframe.");

  return projectLandmarkToCameraFrame(*targetFrame, *sourceFrame, l);
}

Result<Vector2> Graph::projectLandmarkToPixel(
    KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey,
    LandmarkMap::key_type landmarkKey) {
  std::unique_ptr<Frame>& targetFrame = *(keyframes.at(targetFrameKey));
  std::unique_ptr<Frame>& sourceFrame = *(keyframes.at(sourceFrameKey));

  auto landmarkIt = landmarks.at(landmarkKey);
  if (landmarkIt == landmarks.end()) {
    return {};
  }
  Landmark& l = *(landmarkIt);

  CHECK(l.parentFrameKey == sourceFrameKey,
        "Landmark is not child of source keyframe.");

  return projectLandmarkToPixel(*targetFrame, *sourceFrame, l);
}

/// Transforms the landmark into a euclidean point in the current frame.
QDVO::Vector3 Graph::projectLandmarkToCameraFrame(const Frame& targetFrame,
                                                  const Frame& sourceFrame,
                                                  const Landmark& landmark) {
  SE3 T_tf_sf = computeRelativeKeyframeTransform(targetFrame, sourceFrame);
  return T_tf_sf * landmark.getEuclideanPoint();
}

/// Projects a landmark in the pixels in the current frame.
QDVO::Result<QDVO::Vector2> Graph::projectLandmarkToPixel(
    const Frame& targetFrame, const Frame& sourceFrame,
    const Landmark& landmark) {
  auto pt = projectLandmarkToCameraFrame(targetFrame, sourceFrame, landmark);

  std::unique_ptr<CameraModel>& cm =
      cameraModelMap.at(targetFrame.cameraModelKey)->first;

  return cm->project(pt);
}

QDVO::SE3 Graph::computeRelativeKeyframeTransform(const Frame& targetFrame,
                                                  const Frame& sourceFrame) {
  SE3& T_tfimu_tfcam = *(extrinsics.at(targetFrame.extrinsicKey));
  SE3& T_sfimu_sfcam = *(extrinsics.at(sourceFrame.extrinsicKey));

  return (targetFrame.imustate.getSE3() * T_tfimu_tfcam).inverse() *
         (sourceFrame.imustate.getSE3() * T_sfimu_sfcam);
}

QDVO::Result<QDVO::Vector3> Graph::projectLandmarkToOrigin(
    QDVO::Landmark& landmark) {
  auto parentIt = keyframes.at(landmark.parentFrameKey);
  if (parentIt == keyframes.end()) {
    return {};
  }

  auto& parent = *parentIt;

  auto& T_i_c = *extrinsics.at(parent->extrinsicKey);

  return parent->imustate.getSE3() * T_i_c * landmark.getEuclideanPoint();
}

}  // namespace QDVO
