#include "BasicPipeline.h"

#include <algorithm>
#include <cmath>

#include "Config.h"
#include "DataStructures/Feature.h"
#include "DataStructures/GenericQuadTree.h"
#include "DataStructures/Graph.h"
#include "DataStructures/Landmark.h"
#include "EpipolarDepthEstimator.h"
#include "Estimation/FrameToFramePoseEstimator.h"
#include "Profiling.h"

QDVO::BasicPipeline::BasicPipeline() {}

void QDVO::BasicPipeline::initialize() {
  // precompute the radial search pattern LUT
  radialSearchPatternPtr = std::make_shared<RadialSearchPattern>(
      MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS);

  // create the patch warper
  patchWarper = std::make_unique<QDVO::PatchWarper>();

  LOG_INFO("Computed radial search pattern");

  // create a feature detector
  featureDetector = std::make_unique<QDVO::FeatureDetector>();
  LOG_INFO("Created a new feature detector");

  patchComparer = std::make_shared<PatchComparer>();
}

QDVO::CameraModelMap::key_type QDVO::BasicPipeline::addCamera(
    std::unique_ptr<QDVO::CameraModel> cameraModel) {
  QDVO::SE3 unit(Eigen::Quaternion<QDVO::SE3::Scalar>(1, 0, 0, 0),
                 Eigen::Matrix<QDVO::SE3::Scalar, 3, 1>(0, 0, 0));
  LOG_INFO("Camera Model Initialized");
  return graph.insertCamera(std::move(cameraModel), unit);
}

void QDVO::BasicPipeline::addFrame(
    cv::Mat& image, int64_t time,
    const CameraModelMap::key_type& cameraModelKey,
    const ExtrinsicMap::key_type& extrinsicKey) {
  LOG_INFO("Added frame.");
  // save the last imu state
  QDVO::IMUState lastImuState = graph.getCurrentFrame()->imustate;

  // Swap the current and previous frame.
  graph.swapCurrentAndBufferFrame();

  // Reset current frame
  graph.getCurrentFrame()->reset();
  // Setup the current frame.
  {
    PROFILE("updateImage");
    graph.getCurrentFrame()->updateImage(image);
  }

  graph.getCurrentFrame()->status = QDVO::Frame::FrameStatus::INACTIVE;
  graph.getCurrentFrame()->cameraModelKey = cameraModelKey;
  graph.getCurrentFrame()->extrinsicKey = extrinsicKey;
  // for the monocular case, we can assume no motion between frames
  // initially
  graph.getCurrentFrame()->imustate = lastImuState;
  graph.getCurrentFrame()->imustate.time = time;
  graph.getCurrentFrame()->initialized = true;

  // Try to get a coarse initialization for the imustate.
  if (graph.getPreviousFrame()->initialized && false) {
    PROFILE("FrameToFrameRotationEstimator");
    QDVO::FrameToFramePoseEstimator f2fEstimator;
    QDVO::SE3 T_previous_current;
    bool success = f2fEstimator.estimateRelativePose(
        graph, graph.getPreviousFrameKey(), graph.getCurrentFrameKey(),
        T_previous_current);
    LOG_INFO("Estimated Frame2Frame rotation: {}",
             T_previous_current.so3().matrix());

    // Set the initial attitude.
    graph.getCurrentFrame()->imustate.attitude =
        graph.getPreviousFrame()->imustate.attitude * T_previous_current.so3();
  }

  // Initialize correspondence distributions
  {
    PROFILE("initializeCorrespondenceDistribution");
    initializeCorrespondenceDistributionsForCurrentFrame();
  }

  // Run front end visual odometry
  {
    PROFILE("runFrontEndVisualOdometry");
    frontEndVisualOdometry.run(graph);
  }

  // Check if the current frame meets the keyframe selection criteria
  bool isKeyframe = false;
  {
    PROFILE("isCurrentFrameKeyframe");
    isKeyframe = isCurrentFrameAKeyframe();
  }

  if (isKeyframe) {
    // Move the current frame into the keyframe set
    auto newKeyframeKey = graph.getCurrentFrameKey();
    graph.moveCurrentFrameIntoKeyframePosition();
    // The new current frame should not be the same as the old one.
    CHECK(!(newKeyframeKey == graph.getCurrentFrameKey()),
          "Keys were the same.");

    // Cache the imustate from previous current frame estimate.
    auto previousIMUState = graph.getCurrentFrame()->imustate;

    // TODO: The rest of this can be ran on a separate thread.

    // Create new landmarks for the new keyframe
    {
      PROFILE("createNewLandmarks");
      createNewLandmarks(graph, newKeyframeKey, featureDetector);
    }

    // set the new keyframe to active
    (*graph.getKeyframeMap().at(newKeyframeKey))->status = QDVO::Frame::ACTIVE;

    // run the sliding window estimator with the current keyframe
    // set
    {
      PROFILE("runSlidingWindowEstimation");
      swe.run(graph);
    }

    // attempt to estimate the landmark depths using the new motion
    // estimates
    {
      PROFILE("runEpipolarDepthEstimation");
      runEpipolarDepthEstimators(newKeyframeKey);
    }

    // marginalize excess keyframes.
    {
      PROFILE("runMarginalizationStrategy");
      runMarginalizationStrategy();
    }

    // Remove the marginalized keys.
    graph.removeMarginalizedVariables();

    // activate new landmarks if necessary
    {
      PROFILE("activateNewLandmarks");
      activateNewLandmarks();
    }

    // Update the current frame estimate.
    graph.getCurrentFrame()->imustate =
        (*graph.getKeyframeMap().at(newKeyframeKey))->imustate;
  }

  // Determine the current tracking status.
  updateTrackingStatus();

  LOG_INFO("Finished adding frame.");
}

void QDVO::BasicPipeline::updateTrackingStatus() {
  int nActiveKeyframes = 0;
  for (auto& keyframe : graph.getKeyframeMap()) {
    if (keyframe->status == QDVO::Frame::FrameStatus::ACTIVE) {
      ++nActiveKeyframes;
    }
  }

  // If there is only one keyframe, QDVO is still initializing.
  if (nActiveKeyframes == 1) {
    LOG_INFO("Tracking is initializing.");
    status = Status::INITIALIZING;
    return;
  }

  auto latestKeyframeKey = graph.findLatestFrameKey();
  if (latestKeyframeKey.isInvalid()) {
    LOG_ERROR("Latest keyframe is invalid");
    status = Status::LOST_TRACKING;
    return;
  }
  int nVisibleLandmarks = 0;
  for (auto& cd : (*graph.getKeyframeMap().at(latestKeyframeKey))
                      ->correspondenceDistributions) {
    if (cd.initialized) {
      ++nVisibleLandmarks;
    }
  }
  CHECK(frontEndVisualOdometry.lastSolveResult.whitenedSqError.size() > 0,
        "No iterations in FrontEndVisualOdometry.");

  if (nVisibleLandmarks <=
          config->lost_tracking_settings.visibleFeatureThreshold ||
      frontEndVisualOdometry.lastSolveResult.whitenedSqError.back() >
          config->lost_tracking_settings.sqErrorThreshold) {
    LOG_ERROR("Lost tracking!");
    status = Status::LOST_TRACKING;
    return;
  }

  // If we get here, there is tracking.
  status = Status::TRACKING;
}

void QDVO::BasicPipeline::runMarginalizationStrategy() {
  swe.runMarginalizationStrategy(graph);
}

void QDVO::BasicPipeline::createNewLandmarks(
    Graph& graph, KeyframeMap::key_type keyframeKey,
    std::unique_ptr<QDVO::FeatureDetector>& featureDetector) {
  LOG_INFO("Creating new landmarks for keyframe idx:{}, gen:{}",
           keyframeKey.index, keyframeKey.generation);

  std::unique_ptr<Frame>& keyframe = *graph.getKeyframeMap().at(keyframeKey);

  // Detect new features in the keyframe
  std::vector<QDVO::Feature> newFeatures =
      featureDetector->detectFeatures(*keyframe);

  LOG_INFO("Found {} new landmarks", newFeatures.size());

  // add the landmarks to the keyframe's landmark vector
  std::unique_ptr<QDVO::CameraModel>& cm =
      graph.getCameraModelMap().at(keyframe->cameraModelKey)->first;

  for (auto& f : newFeatures) {
    Landmark lm;
    lm.px = Eigen::Matrix<SCALAR_TYPE, 2, 1>(f.px.x, f.px.y);
    lm.parentFrameKey = keyframeKey;
    lm.dinv = DEFAULT_LANDMARK_DINV;

    auto result = cm->unproject(lm.px);
    if (!result.has_value()) {
      LOG_INFO("failed to unproject pixel.");
      continue;
    }

    lm.bearing = result.value();

    auto landmarkKey = graph.getLandmarkMap().insert(lm);

    keyframe->landmarkKeys.push_back(landmarkKey);
  }
}

void QDVO::BasicPipeline::
    initializeCorrespondenceDistributionsForCurrentFrame() {
  LOG_INFO("Initializing correspondence distributions for current frame.");
  // first update the patch comparers before initializing all the
  // correspondence distributions
  updatePatchComparers();

  std::unique_ptr<QDVO::Frame>& cf = graph.getCurrentFrame();
  std::unique_ptr<CameraModel>& cm =
      graph.getCameraModelMap().at(cf->cameraModelKey)->first;

  // second reset correspondence distributions
  cf->resetCorrespondenceDistributions();

  // find the set of active landmarks visible in the current frame.
  // create and initialize the correspondence distribution for each of
  // these landmarks
  std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>>
      visibleActiveLandmarks = graph.getVisibleLandmarksInCurrentFrame(true);

  LOG_INFO("found {} visible and active landmarks for the current frame",
           visibleActiveLandmarks.size());

  // A list of all keys pushed into the frame.
  std::vector<CorrespondenceDistributionMap::key_type>
      correspondenceDistributionKeys;

  for (int i = 0; i < visibleActiveLandmarks.size(); ++i) {
    // create another correspondence distribution
    correspondenceDistributionKeys.push_back(
        cf->correspondenceDistributions.insert(QDVO::CorrespondenceDistribution(
            cm->width, cm->height, radialSearchPatternPtr)));
  }

  auto initializationFn =
      [&](std::tuple<LandmarkMap::key_type, QDVO::Vector2>& tup,
          CorrespondenceDistributionMap::key_type cdKey) -> int {
    LandmarkMap::key_type lKey = std::get<0>(tup);

    Landmark& l = *graph.getLandmarkMap().at(lKey);

    Frame& f = *(*graph.getKeyframeMap().at(l.parentFrameKey));

    CameraModel& cm = *(graph.getCameraModelMap().at(f.cameraModelKey)->first);

    auto& cdRef = *cf->correspondenceDistributions.at(cdKey);
    CHECK(cdRef.initialized == false,
          "The correspondence distribution must not be initialized");

    Frame& landmarkParentFrame =
        *(*graph.getKeyframeMap().at(l.parentFrameKey));

    // initialize the correspondence distribution
    auto px0 = graph.projectLandmarkToPixel(*graph.getCurrentFrame(),
                                            landmarkParentFrame, l);
    if (!px0.has_value()) {
      LOG_TRACE("landmark not visible in its parent frame!");
      return 1;
    }
    QDVO::Result<QDVO::Patch> warpedPatch = {};

    // warp the patch.
    patchWarper->warpPatchToTargetFrame(warpedPatch, l, landmarkParentFrame,
                                        *(graph.getCurrentFrame()), graph);
    if (!warpedPatch.has_value()) {
      LOG_TRACE("failed to warp patch");
      return 1;
    }

    cdRef.initializeDistribution(
        cm, *(graph.getCurrentFrame()), lKey,
        Eigen::Vector2i(std::round(px0.value()(0)), std::round(px0.value()(1))),
        MAXIMUM_CORRESPONDENCE_SEARCH_RADIUS, patchComparer,
        warpedPatch.value());

    // If this landmark was observed increment the observation counter.
    if (cdRef.initialized) {
      ++l.nObservations;
    }

    return 0;
  };

  std::vector<int> result(visibleActiveLandmarks.size());
  // Run the initialization function for all active and visible landmarks.
  QDVO::ParallelAlgorithms::transform(
      QDVO::ParallelAlgorithms::ExecutionType::SEQUENTIAL,
      visibleActiveLandmarks.begin(), visibleActiveLandmarks.end(),
      correspondenceDistributionKeys.begin(), result.begin(), initializationFn);

  LOG_INFO(
      "Initialized correspondence distributions for this frame. Could "
      "not initialize: {} distributions",
      std::accumulate(result.begin(), result.end(), 0));
}

bool QDVO::BasicPipeline::isCurrentFrameAKeyframe() {
  if (graph.getKeyframeMap().size() == 1) {
    LOG_INFO("First frame is always a keyframe.");
    return true;
  }

  // Compute the pixel flow for each keyframe to check if the current
  // frame should be made a keyframe. Find the lowest pixel flow score.
  double smallestPixelFlowScore = std::numeric_limits<double>::max();
  for (auto it = graph.getKeyframeMap().begin();
       it != graph.getKeyframeMap().end(); it++) {
    auto thisKey = graph.getKeyframeMap().getKeyFromDataIndex(
        it - graph.getKeyframeMap().begin());
    if (thisKey == graph.getCurrentFrameKey() ||
        (*it)->status == QDVO::Frame::FrameStatus::INACTIVE) {
      continue;
    }

    // Compute the pixel flow with the current frame.
    auto pixelFlows = computePixelFlowForCurrentFrame(thisKey);

    LOG_INFO(
        "For keyframe {}-{}, Average pixel flow: {} Average "
        "translational pixel flow: {}",
        thisKey.index, thisKey.generation, pixelFlows.first, pixelFlows.second);

    double score = config->weightAvgPixelFlow * pixelFlows.first +
                   config->weightAvgTranslationalFlow * pixelFlows.second;

    if (score < smallestPixelFlowScore) {
      smallestPixelFlowScore = score;
    }
  }

  assert(!std::isnan(smallestPixelFlowScore));
  // If this condition is met for any keyframe, The current frame pose is
  // sufficiently "far" from all keyframes to warrant creating a new
  // keyframe.
  if (smallestPixelFlowScore > 1) {
    LOG_INFO("Current frame qualifies as a keyframe!");
    return true;
  }

  return false;
}

std::pair<double, double> QDVO::BasicPipeline::computePixelFlowForCurrentFrame(
    KeyframeMap::key_type key) {
  Frame& latestKeyframe = *(*graph.getKeyframeMap().at(key));
  QDVO::SE3 currentFramePose = graph.getCurrentFrame()->imustate.getSE3();
  QDVO::SE3 latestKeyframePose = latestKeyframe.imustate.getSE3();
  double pixelFlow = 0;
  int numberOfObservedFeatures = 0;
  for (auto landmarkIt = graph.getLandmarkMap().begin();
       landmarkIt != graph.getLandmarkMap().end(); landmarkIt++) {
    // Skip this landmark if it is not active.
    if (landmarkIt->status != Landmark::LandmarkStatus::ACTIVE) {
      continue;
    }

    auto lKey = graph.getLandmarkMap().getKeyFromDataIndex(
        landmarkIt - graph.getLandmarkMap().begin());

    auto keyframePixel =
        graph.projectLandmarkToPixel(key, landmarkIt->parentFrameKey, lKey);
    auto currentFramePixel = graph.projectLandmarkToPixel(
        graph.getCurrentFrameKey(), landmarkIt->parentFrameKey, lKey);

    if (keyframePixel.has_value() && currentFramePixel.has_value()) {
      pixelFlow += (currentFramePixel.value() - keyframePixel.value()).norm();
      ++numberOfObservedFeatures;
    }
  }

  pixelFlow = pixelFlow / numberOfObservedFeatures;

  double translationalPixelFlow = 0;
  // Set the current frame rotation to the keyframe rotation
  graph.getCurrentFrame()->imustate.attitude = latestKeyframePose.so3();
  numberOfObservedFeatures = 0;
  for (auto landmarkIt = graph.getLandmarkMap().begin();
       landmarkIt != graph.getLandmarkMap().end(); landmarkIt++) {
    // Skip this landmark if it is not active.
    if (landmarkIt->status != Landmark::LandmarkStatus::ACTIVE) {
      continue;
    }

    auto lKey = graph.getLandmarkMap().getKeyFromDataIndex(
        landmarkIt - graph.getLandmarkMap().begin());

    auto keyframePixel =
        graph.projectLandmarkToPixel(key, landmarkIt->parentFrameKey, lKey);
    auto currentFramePixel = graph.projectLandmarkToPixel(
        graph.getCurrentFrameKey(), landmarkIt->parentFrameKey, lKey);

    if (keyframePixel.has_value() && currentFramePixel.has_value()) {
      translationalPixelFlow +=
          (currentFramePixel.value() - keyframePixel.value()).norm();
      ++numberOfObservedFeatures;
    }
  }
  // Reset the current frame attitude
  graph.getCurrentFrame()->imustate.attitude = currentFramePose.so3();

  translationalPixelFlow = translationalPixelFlow / numberOfObservedFeatures;

  return std::make_pair(pixelFlow, translationalPixelFlow);
}

void QDVO::BasicPipeline::updatePatchComparers() {}

void QDVO::BasicPipeline::runEpipolarDepthEstimators(
    KeyframeMap::key_type mostRecentKeyframeKey) {
  LOG_INFO("Running Epipolar Depth Estimators");
  // create a shared pointer to the patch comparer.
  auto patchComparerPtr = patchComparer;

  Frame& targetKeyframe = *(*graph.getKeyframeMap().at(mostRecentKeyframeKey));
  // Iterate through all active keyframes and look for uninitialized
  // landmarks.
  for (auto keyframeIt = graph.getKeyframeMap().begin();
       keyframeIt != graph.getKeyframeMap().end(); keyframeIt++) {
    Frame& sourceKeyframe = *(*keyframeIt);
    auto sourceKeyframeKey = graph.getKeyframeMap().getKeyFromDataIndex(
        keyframeIt - graph.getKeyframeMap().begin());

    // Check if this keyframe is active and has parallax with the
    // most recent keyframe.
    if (sourceKeyframe.status == Frame::FrameStatus::ACTIVE &&
        !(sourceKeyframeKey == mostRecentKeyframeKey)) {
      // Iterate through all hosted landmarks in this
      // keyframe.
      for (auto landmarkKey : sourceKeyframe.landmarkKeys) {
        // Get the landmark.
        auto landmarkIt = graph.getLandmarkMap().at(landmarkKey);
        if (landmarkIt == graph.getLandmarkMap().end()) {
          LOG_TRACE("keyframe contained invalid landmark key");
          continue;
        }
        auto& landmark = *(landmarkIt);

        // Check if the landmark should be updated.
        if (!landmark.depthEstimator.initialized &&
            landmark.status == Landmark::LandmarkStatus::INACTIVE) {
          // Update this landmark's depth
          // estimator.
          landmark.depthEstimator.update(graph, sourceKeyframe, targetKeyframe,
                                         landmark, *patchComparerPtr,
                                         *patchWarper);
        }
      }
    }
  }
  LOG_INFO("Finished running Epipolar Depth Estimators.");
}

void QDVO::BasicPipeline::activateNewLandmarks() {
  LOG_INFO("Activating landmarks.");

  // find all visible active and inactive landmarks
  std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>>
      visibleLandmarks = graph.getVisibleLandmarksInCurrentFrame(false, true);

  LOG_INFO(
      "there are currently {} active and inactive landmarks visible in "
      "the current frame",
      visibleLandmarks.size());

  auto& cm = graph.getCameraModelMap()
                 .at(graph.getCurrentFrame()->cameraModelKey)
                 ->first;

  // Create a quadtree for fast nearest neighbor searches.
  QDVO::GenericQuadTree<QDVO::Vector2> tree(cm->imageWidth(),
                                            cm->imageHeight());

  int nActiveLandmarks = 0;

  //
  //
  // Fill the quad tree with active landmarks.
  //
  //
  for (auto& t : visibleLandmarks) {
    LandmarkMap::key_type lKey = std::get<0>(t);

    Landmark& l = *graph.getLandmarkMap().at(lKey);

    if (l.status == QDVO::Landmark::ACTIVE) {
      tree.insert(std::get<1>(t).cast<int>(), std::get<1>(t));
      ++nActiveLandmarks;
    }
  }

  LOG_INFO("{} Active visible landmarks before activation.", nActiveLandmarks);

  if (nActiveLandmarks >= N_ACTIVE_LANDMARKS_DESIRED) {
    return;
  }

  // Lambda which computes the smallest distance between a point and point set.
  auto minimumDistance = [](const std::vector<QDVO::Vector2>& points,
                            const QDVO::Vector2& testPoint) {
    double smallestDistance = std::numeric_limits<double>::max();
    for (const auto& point : points) {
      double distance = (point - testPoint).norm();
      if (distance < smallestDistance) {
        smallestDistance = distance;
      }
    }
    return smallestDistance;
  };

  // Get a set of visible inactive initialized landmarks.
  std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>>
      inactiveInitializedVisibleLandmarks;
  // Get a set of visible inactive uninitialized landmarks.
  std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>>
      inactiveUninitializedVisibleLandmarks;

  // Extract the two subsets.
  for (auto& t : visibleLandmarks) {
    LandmarkMap::key_type lKey = std::get<0>(t);

    Landmark& l = *graph.getLandmarkMap().at(lKey);

    if (l.status == QDVO::Landmark::INACTIVE) {
      if (l.depthEstimator.initialized) {
        inactiveInitializedVisibleLandmarks.push_back(t);
      } else {
        inactiveUninitializedVisibleLandmarks.push_back(t);
      }
    }
  }

  // Normalizing factors.
  const double maxPixelDistance = Eigen::Vector2d(cm->width, cm->height).norm();
  const double maxDepthError = config->epipolar_depth_estimator.maximumDepth -
                               config->epipolar_depth_estimator.minimumDepth;

  // Lambda which selects the best initialized active landmark.
  auto selectBestLandmark =
      [&](const std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>>&
              choices,
          bool weightOnError = false)
      -> QDVO::Result<std::tuple<LandmarkMap::key_type, QDVO::Vector2>> {
    // Make an empty result.
    QDVO::Result<std::tuple<LandmarkMap::key_type, QDVO::Vector2>> result;
    double largestDistance = -1;
    // Iterate through all choices to find the best option.
    for (const auto& t : choices) {
      LandmarkMap::key_type lKey = std::get<0>(t);
      Landmark& l = *graph.getLandmarkMap().at(lKey);
      // Check if the landmark is still inactive.
      if (l.status == Landmark::LandmarkStatus::INACTIVE) {
        // Compute the landmarks distance to the nearest active landmark.
        auto neighbors =
            tree.getNeighbors(std::get<QDVO::Vector2>(t).cast<int>(), 4, 1);
        // If there are no neighbors, add this one.
        if (neighbors.empty()) {
          result = t;
          return result;
        }

        double nearestNeighborDistance =
            minimumDistance(neighbors, std::get<QDVO::Vector2>(t));
        // Skip if this landmark is too close to a neighbor.
        if (nearestNeighborDistance < MINUMUM_LANDMARK_SEPERATION) {
          continue;
        }

        // If we should weight on error, normalize the distance.
        if (weightOnError) {
          double neighborDistNormalized =
              nearestNeighborDistance / maxPixelDistance;
          // Inverted noramlized depth error.
          double normalizedDepthError = std::abs(
              1 - std::clamp(l.depthEstimator.error * l.dinv / maxDepthError,
                             0.0, 1.0));

          CHECK(neighborDistNormalized <= 1 && neighborDistNormalized >= 0,
                "The distance is not bounded.")
          CHECK(normalizedDepthError <= 1 && normalizedDepthError >= 0,
                "The depth error is not bounded.")

          // Compute the weighted "distance".
          nearestNeighborDistance =
              neighborDistNormalized *
                  (1 - config->activation_settings.depthErrorBias) +
              normalizedDepthError * config->activation_settings.depthErrorBias;
        }

        // If this landmark is far enough and the farthest so far, make it the
        // result.
        if (nearestNeighborDistance > largestDistance) {
          result = t;
          largestDistance = nearestNeighborDistance;
        }
      }
    }
    return result;
  };

  //
  //
  // Activate as many initilaized landmarks as possible.
  //
  //
  //
  auto activateLandmarks =
      [&](const std::vector<std::tuple<LandmarkMap::key_type, QDVO::Vector2>>&
              choices,
          int stopAfterNActiveLandmarks, bool weightOnDepthError = false) {
        bool shouldContinue = false;
        do {
          // Break if we have enough active landmarks.
          if (nActiveLandmarks >= stopAfterNActiveLandmarks) {
            break;
          }

          // Find a good landmark.
          auto result = selectBestLandmark(choices, weightOnDepthError);

          if (result.has_value()) {
            shouldContinue = true;

            // Activate the desired landmark.
            LandmarkMap::key_type lKey = std::get<0>(result.value());
            Landmark& l = *graph.getLandmarkMap().at(lKey);
            l.status = Landmark::LandmarkStatus::ACTIVE;
            // Add the landmark to the quadtree.
            tree.insert(std::get<QDVO::Vector2>(result.value()).cast<int>(),
                        std::get<QDVO::Vector2>(result.value()));
            // Increment the active landmark counter.
            ++nActiveLandmarks;

            LOG_INFO(
                "Activating landmark with depth: {} depth error: {} "
                "hypotheses: {} epipolar depth updates: {}",
                1.0 / l.dinv, l.depthEstimator.error,
                l.depthEstimator.hypotheses, l.depthEstimator.attempts);

          } else {
            shouldContinue = false;
          }
        } while (shouldContinue);
      };

  activateLandmarks(inactiveInitializedVisibleLandmarks,
                    N_ACTIVE_LANDMARKS_DESIRED);

  LOG_INFO(
      "{} Active visible landmarks after activating initialized "
      "landmarks.",
      nActiveLandmarks);

  // if necessary activate uninitialized landmarks
  if (nActiveLandmarks < MINUMUM_ACTIVE_LANDMARKS) {
    LOG_WARN(
        "Too few active landmarks, activating unintialized landmarks. This may "
        "cause tracking loss.");

    // Start with the set of landmarks observed atleast once.
    activateLandmarks(inactiveUninitializedVisibleLandmarks,
                      MINUMUM_ACTIVE_LANDMARKS, true);

    LOG_INFO(
        "{} active visible landmarks after activating "
        "uninitialized landmarks.",
        nActiveLandmarks);
  }
  LOG_INFO("Finished activating landmakrs.");
}
