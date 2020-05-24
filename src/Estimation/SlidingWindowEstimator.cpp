#include "SlidingWindowEstimator.h"

#include "Logging.h"

using namespace QDVO;

SlidingWindowEstimator::SlidingWindowEstimator() {}

void SlidingWindowEstimator::run(QDVO::Graph& graph) {
  // Check if a new keyframe has been added.
  LOG_TRACE(
      "Checking if a new active keyframe has been added to the graph. Current "
      "number of keyframes in the graph: {}",
      graph.getKeyframeMap().size());

  for (auto it = graph.getKeyframeMap().begin();
       it != graph.getKeyframeMap().end(); it++) {
    auto keyframeKey = graph.getKeyframeMap().getKeyFromDataIndex(
        it - graph.getKeyframeMap().begin());

    // Check if this keyframe is the current frame or it is not active.
    if (keyframeKey == graph.getCurrentFrameKey() ||
        (*it)->status != Frame::FrameStatus::ACTIVE) {
      // Skip this frame.
      LOG_TRACE(
          "Skipping frame because it is either the current frame or inactive. "
          "Frame state: {}",
          (*it)->status);
      continue;
    }

    // Check if we have not added this frame before.
    bool keyframeNotInSWE = true;
    for (auto innerIt = poseKeyMap.begin(); innerIt != poseKeyMap.end();
         innerIt++) {
      auto key = poseKeyMap.getKeyFromDataIndex(innerIt - poseKeyMap.begin());
      if (key == keyframeKey) {
        keyframeNotInSWE = false;
      }
    }
    // If the keyframe is new, add it and all its landmarks to the SWE.
    if (keyframeNotInSWE) {
      LOG_TRACE("Found new keyframe for sliding window estimator.");
      // Insert a variable for the keyframe pose.
      ArgMin::SE3 pose;
      pose.value = (*it)->imustate.getSE3();
      auto poseKey = variableContainer.insert(pose);
      // Insert a map from the variable key to keyframe key.
      poseKeyMap.insert(keyframeKey, poseKey);
      // Insert the variable into the prior.
      if (poseKeyMap.size() == 1) {
        LOG_TRACE("Adding first keyframe pose with strong prior.");
        prior.addVariable(
            poseKey, Eigen::Matrix<double, 6, 1>::Constant(1e24).asDiagonal());
      } else {
        prior.addVariable(poseKey);
      }

      LOG_TRACE("Added new keyframe to estimator.");
      LOG_TRACE("Constructing error terms for new keyframe.");

      /// Add a landmark to the estimator if necessary.
      /// Returns true if the landmark was added.
      auto addLandmarkToEstimatorIfNecessary =
          [&](LandmarkMap::key_type landmarkKey) -> bool {
        // Return early if the variable already exists.
        if (dinvKeyMap.at(landmarkKey) != dinvKeyMap.end()) {
          return false;
        }
        // Get the landmark.
        const auto& landmark = *graph.getLandmarkMap().at(landmarkKey);

        // make sure that the landmark is active.
        CHECK(landmark.status == Landmark::LandmarkStatus::ACTIVE,
              "The landmark must be active.");

        // Insert an inverse depth variable into the estimator.
        ArgMin::InverseDepth dinv;
        dinv.value = landmark.dinv;
        auto dinvKey = variableContainer.insert(dinv);
        // Insert mapping between variable and landmark key.
        dinvKeyMap.insert(landmarkKey, dinvKey);
        // Insert variable into the prior.
        prior.addVariable(dinvKey);
        return true;
      };

      // Iterate through the correspondence distributions in this
      // keyframe, and add them as error terms.
      int cdIdx = 0;
      int errorTermsForThisKeyframe = 0;
      for (auto cdIt = (*it)->correspondenceDistributions.begin();
           cdIt != (*it)->correspondenceDistributions.end(); cdIt++) {
        auto& cd = *cdIt;
        if (!cd.initialized) {
          LOG_TRACE(
              "Skipping correspondence distribution because it was not "
              "initialized.");
          continue;
        }

        // Add the variable to the estimator if necessary.
        if (addLandmarkToEstimatorIfNecessary(cd.landmarkKey)) {
          LOG_TRACE("Added landmark {}-{} to the Sliding Window Estimator",
                    cd.landmarkKey.index, cd.landmarkKey.generation);
        }

        auto targetFramePoseVariableKey = *poseKeyMap.at(keyframeKey);
        assert(targetFramePoseVariableKey == poseKey);
        auto observedLandmarkIt = graph.getLandmarkMap().at(cd.landmarkKey);
        assert(observedLandmarkIt != graph.getLandmarkMap().end());
        auto sourceFramePoseVariableKey =
            *poseKeyMap.at(observedLandmarkIt->parentFrameKey);
        auto dinvVariableKey = *dinvKeyMap.at(cd.landmarkKey);

        QuasiDirectErrorTerm qdet(
            sourceFramePoseVariableKey, targetFramePoseVariableKey,
            dinvVariableKey, Eigen::Matrix<double, 2, 2>::Identity(),
            (*it)->correspondenceDistributions.getKeyFromDataIndex(
                std::distance((*it)->correspondenceDistributions.begin(),
                              cdIt)),
            &graph, observedLandmarkIt->parentFrameKey, keyframeKey,
            cd.landmarkKey);

        errorTermContainer.insert(qdet);

        // Increment the correspondence distribution index.
        ++errorTermsForThisKeyframe;
      }
      LOG_TRACE("Constructed {} error terms for the new keyframe",
                errorTermsForThisKeyframe);
    }
  }

  // Clean the prior before optimizing.
  prior.removeUnsedVariables(variableContainer);

  // Run the solver.
  auto result = solver.solveLevenbergMarquardt(variableContainer,
                                               errorTermContainer, prior);

  // Set the last solve result.
  lastSolveResult = result;

  LOG_INFO("Solver ran for {} iterations.", result.whitenedSqError.size());
  if (!result.whitenedSqError.empty()) {
    LOG_INFO("Initial squared error {} -> final squared error {}",
             result.whitenedSqError.front(), result.whitenedSqError.back());
  }

  // Apply the updates to the actual graph iff the error was decreased.
  if (!result.whitenedSqError.empty() &&
      result.whitenedSqError.back() < result.whitenedSqError.front()) {
    LOG_INFO("SWE successfully reduced error, applying updates to graph.");
    synchronizeGraph(graph);

    // remove outliers found during sliding window estimation
    LOG_INFO("Removing outliers after successful optimization");
    removeOutliers(graph);
  } else {
    LOG_ERROR("Error increased, still syncing update with graph.");
    synchronizeGraph(graph);

    // remove outliers found during sliding window estimation
    LOG_INFO("Removing outliers after unsuccessful optimization");
    removeOutliers(graph);
  }
}

void SlidingWindowEstimator::removeOutliers(QDVO::Graph& graph) {
  // Iterate through all visual error terms and check if any residuals are too
  // high.
  std::vector<LandmarkMap::key_type> landmarksToMarginalize;
  auto& errorTermMap =
      errorTermContainer.getErrorTermMap<QDVO::QuasiDirectErrorTerm>();
  for (auto& errorTerm : errorTermMap) {
    if (errorTerm.residual.norm() >= config->pixelOutlierThreshold) {
      landmarksToMarginalize.push_back(errorTerm.landmarkKey);
    }
  }

  // Marginalize all landmarks which are outliers.
  int marginalizedLandmarks = 0;
  for (auto& landmarkKey : landmarksToMarginalize) {
    if (graph.getLandmarkMap().at(landmarkKey)->status !=
        Landmark::LandmarkStatus::MARGINALIZED) {
      marginalizeLandmark(graph, landmarkKey);
      ++marginalizedLandmarks;
    }
  }

  LOG_INFO("Remove {} outlier landmarks.", marginalizedLandmarks);
}

void SlidingWindowEstimator::runMarginalizationStrategy(QDVO::Graph& graph) {
  LOG_TRACE("Running marginalization strategy.");
  if (graph.getKeyframeMap().size() <= N_KEYFRAMES) {
    LOG_TRACE("There are {} keyframes, no need to marginalize one.",
              graph.getKeyframeMap().size());
    return;
  }

  // Step 0
  // Create a list of active keyframe keys sorted by time.
  std::vector<KeyframeMap::key_type> activeKeyframeKeys;
  for (auto it = graph.getKeyframeMap().begin();
       it != graph.getKeyframeMap().end(); it++) {
    if ((*it)->status == Frame::FrameStatus::ACTIVE) {
      activeKeyframeKeys.push_back(graph.getKeyframeMap().getKeyFromDataIndex(
          std::distance(graph.getKeyframeMap().begin(), it)));
    }
  }

  // Sort the keys by the keyframe time in ascending order.
  std::sort(activeKeyframeKeys.begin(), activeKeyframeKeys.end(),
            [&graph](auto a, auto b) {
              return (*graph.getKeyframeMap().at(a))->imustate.time <
                     (*graph.getKeyframeMap().at(b))->imustate.time;
            });

  CHECK(
      (*graph.getKeyframeMap().at(activeKeyframeKeys.front()))->imustate.time <
          (*graph.getKeyframeMap().at(activeKeyframeKeys.back()))
              ->imustate.time,
      "Times are not in order.");

  CHECK(activeKeyframeKeys.size() > 2,
        "There must be at least 3 keyframes to run the marginalizer!");

  // Step 1
  // Check if any keyframes have fewer than N% of the total active landmarks
  // visible.
  constexpr double activeLandmarkRatioThreshold = 0.02;

  // Compute the number of visible landmarks in the newest keyframe and the
  // number of visible landmarks hosted by each frame.
  ArgMin::SlotArray<int, QDVO::KeyframeMap::key_type>
      visibleLandmarksHostedInEachFrame;
  // Fill the slot map.
  for (auto key : activeKeyframeKeys) {
    visibleLandmarksHostedInEachFrame.insert(key, 0);
  }

  int nActiveVisibleLandmarksInCurrentFrame = 0;
  // Get the newest keyframe key
  auto newestKeyframeKey = activeKeyframeKeys.back();
  // Get the newest keyframe.
  auto& newestKeyframe =
      *(*(graph.getKeyframeMap().at(activeKeyframeKeys.back())));

  for (const auto& correspondenceDistribution :
       newestKeyframe.correspondenceDistributions) {
    // If the correspondence dist is initialized, check if the landmark it is
    // associated to is initialized.
    if (correspondenceDistribution.initialized) {
      auto landmarkIt =
          graph.getLandmarkMap().at(correspondenceDistribution.landmarkKey);
      // If the landmark is active, increment the observation counters.
      if (landmarkIt != graph.getLandmarkMap().end()) {
        const auto& landmark = *landmarkIt;
        // Check if the landmark is active.
        if (landmark.status == Landmark::LandmarkStatus::ACTIVE) {
          // Increment the total number of visible active landmarks.
          ++nActiveVisibleLandmarksInCurrentFrame;

          // Increment the landmarks parent frame counter.
          *(visibleLandmarksHostedInEachFrame.at(landmark.parentFrameKey)) += 1;
        }
      }
    }
  }

  // Find if any keyframes contain too few active visible landmarks.
  for (auto frameKeyIt = activeKeyframeKeys.begin();
       frameKeyIt != activeKeyframeKeys.end() - 2; frameKeyIt++) {
    auto frameKey = *frameKeyIt;
    int nLandmarksInThisFrame = *visibleLandmarksHostedInEachFrame.at(frameKey);
    double ratio = (double)nLandmarksInThisFrame /
                   (double)nActiveVisibleLandmarksInCurrentFrame;
    LOG_TRACE(
        "Frame {} contains {} active and visible landmarks out of the total {} "
        "active and visible landmarks. Ratio: {}",
        frameKey.index, nLandmarksInThisFrame,
        nActiveVisibleLandmarksInCurrentFrame, ratio);

    if (ratio < activeLandmarkRatioThreshold) {
      LOG_TRACE(
          "Marginalizing keyframe {}-{} with too few active visible landmarks.",
          frameKey.index, frameKey.generation);
      // Marginalize.
      marginalizeKeyframe(graph, frameKey);
      // Return because we are done.
      return;
    }
  }

  // Step 2
  // If we could not find any weakly connected keyframes to marginalize, try to
  // maximize the spatial districution of the keyframes.

  // Create a distance score map.
  ArgMin::SlotArray<double, KeyframeMap::key_type> distanceScores;
  // Epsilon for distance computation.
  constexpr double epsilon = 1e-16;

  // Iterate through all but the two newest keyframes.
  for (auto frameKeyIt = activeKeyframeKeys.begin();
       frameKeyIt != activeKeyframeKeys.end() - 2; frameKeyIt++) {
    auto frameKey = *frameKeyIt;
    auto& outerFrame = *(*graph.getKeyframeMap().at(frameKey));

    // Compute the distance between this frame and the newest keyframe.
    double d_i_1 =
        (outerFrame.imustate.pos - newestKeyframe.imustate.pos).norm();

    // Insert a distance score for this keyframe.
    distanceScores.insert(frameKey, 0);

    // Now iterate through all keyframes again.
    for (auto innerFrameKeyIt = activeKeyframeKeys.begin();
         innerFrameKeyIt != activeKeyframeKeys.end() - 2; innerFrameKeyIt++) {
      auto innerFrameKey = *innerFrameKeyIt;
      // Don't use the same keyframe.
      if (!(innerFrameKey == frameKey)) {
        auto& innerFrame = *(*graph.getKeyframeMap().at(innerFrameKey));
        double d_i_j =
            (outerFrame.imustate.pos - innerFrame.imustate.pos).norm();

        // Get the distance score.;
        auto& score = *distanceScores.at(frameKey);
        score += 1.0 / (d_i_j + epsilon);
      }
    }

    // Finalize the score.
    auto& score = *distanceScores.at(frameKey);
    score = sqrt(d_i_1) * score;
  }

  // Select the maximum distance keyframe.
  auto selectedKeyframeKeyIt =
      std::max_element(distanceScores.begin(), distanceScores.end());

  CHECK(selectedKeyframeKeyIt != distanceScores.end(), "No maximum score.");
  auto keyToMarginalize = distanceScores.getKeyFromDataIndex(
      std::distance(distanceScores.begin(), selectedKeyframeKeyIt));

  LOG_TRACE("Selected keyframe {}-{} with score {} for marginalization",
            keyToMarginalize.index, keyToMarginalize.generation,
            *selectedKeyframeKeyIt);

  marginalizeKeyframe(graph, keyToMarginalize);
}

void SlidingWindowEstimator::synchronizeGraph(QDVO::Graph& graph) {
  for (auto it = poseKeyMap.begin(); it != poseKeyMap.end(); it++) {
    auto key = poseKeyMap.getKeyFromDataIndex(it - poseKeyMap.begin());
    auto& keyframe = (*graph.getKeyframeMap().at(key));
    auto& variable = variableContainer.at(*it);
    keyframe->imustate.pos = variable.value.translation();
    keyframe->imustate.attitude = variable.value.so3();
  }

  for (auto it = dinvKeyMap.begin(); it != dinvKeyMap.end(); it++) {
    auto key = dinvKeyMap.getKeyFromDataIndex(it - dinvKeyMap.begin());
    // Get the landmark.
    auto landmarkIt = graph.getLandmarkMap().at(key);
    if (landmarkIt == graph.getLandmarkMap().end()) {
      LOG_TRACE("inversedepth key map contained invalid landmark key");
      continue;
    }
    auto& landmark = *(landmarkIt);
    auto& variable = variableContainer.at(*it);
    landmark.dinv = variable.value;
  }
}

void SlidingWindowEstimator::marginalizeLandmark(
    QDVO::Graph& graph, LandmarkMap::key_type landmarkKey) {
  auto landmarkIt = graph.getLandmarkMap().at(landmarkKey);
  if (landmarkIt == graph.getLandmarkMap().end()) {
    LOG_ERROR("Tried to marginalize nonexistent landmark");
    return;
  }
  auto& landmark = *landmarkIt;
  auto variableIt = dinvKeyMap.at(landmarkKey);
  if (variableIt == dinvKeyMap.end()) {
    LOG_ERROR("Could not marginalize variable key.");
    // Set the landmark to marginalized.
    landmark.status = Landmark::LandmarkStatus::MARGINALIZED;
    return;
  }
  auto variableKey = *variableIt;

  if (landmark.status == Landmark::LandmarkStatus::ACTIVE) {
    CHECK(variableContainer.variableExists(variableKey),
          "Landmark is active, but the variable container does not have it.");

    // First marginalize the landmark.
    marginalizer.marginalizeVariable(variableKey, prior, errorTermContainer,
                                     ArgMin::VariableGroup<>(),
                                     config->pixelOutlierThreshold);
  } else {
    LOG_WARN("Tried to marginalized landmark with status: {}", landmark.status);
  }
  // Set the landmark to marginalized.
  landmark.status = Landmark::LandmarkStatus::MARGINALIZED;
  // Delete the landmark variable from the SWE.
  variableContainer.erase(variableKey);
  dinvKeyMap.erase(landmarkKey);
}

void SlidingWindowEstimator::marginalizeKeyframe(
    QDVO::Graph& graph, KeyframeMap::key_type keyframeKey) {
  // First marginalize all landmarks one at a time.
  auto& keyframe = *(*graph.getKeyframeMap().at(keyframeKey));
  CHECK(keyframe.status == Frame::FrameStatus::ACTIVE,
        "The keyframe must be active to be marginalized.");
  for (auto& landmarkKey : keyframe.landmarkKeys) {
    marginalizeLandmark(graph, landmarkKey);
  }

  // Finally marginalize the keyframe itself while ignoring landmark
  // correlations to preserve sparsity.
  auto variableKey = *poseKeyMap.at(keyframeKey);

  // The variable must exist in the variable container.
  CHECK(variableContainer.variableExists(variableKey),
        "Keyframe is active, but the variable container does not have it.");

  marginalizer.marginalizeVariable(
      variableKey, prior, errorTermContainer,
      ArgMin::VariableGroup<ArgMin::InverseDepth>(),
      config->pixelOutlierThreshold);
  variableContainer.erase(variableKey);
  poseKeyMap.erase(keyframeKey);
  // Set the keyframe to marginalized.
  keyframe.status = Frame::FrameStatus::MARGINALIZED;
}
