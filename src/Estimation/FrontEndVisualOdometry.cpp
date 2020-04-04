#include "FrontFndVisualOdometry.h"
#include "Logging.h"

using namespace QDVO;

FrontEndVisualOdometry::FrontEndVisualOdometry() {
  ArgMin::SE3 pose;
  poseKey = variableContainer.insert(std::move(pose));
  prior.addVariable(poseKey);
}

void FrontEndVisualOdometry::run(QDVO::Graph& graph) {
  LOG_INFO("Clearing old error terms.");
  // Ensure that the error term container is clear.
  errorTermContainer.clear();

  LOG_INFO("Creating error terms for the current frame.");
  Frame& currentFrame = *graph.getCurrentFrame();
  int nErrorTerms = 0;
  for (auto it = currentFrame.correspondenceDistributions.begin();
       it != currentFrame.correspondenceDistributions.end(); it++) {
    if (!it->initialized) {
      LOG_TRACE(
          "Skipping correspondence distribution because it was not "
          "initialized.");
      continue;
    }
    auto landmark = graph.getLandmarkMap().at(it->landmarkKey);

    if (landmark->status != Landmark::LandmarkStatus::ACTIVE) {
      LOG_WARN(
          "Invalid landmark was in the correspondence distribution list {}",
          landmark->status);
      continue;
    }

    errorTermContainer.insert(QuasiDirectErrorTerm_TargetFrame(
        poseKey, Eigen::Matrix2d::Identity(), &graph, landmark->parentFrameKey,
        graph.getCurrentFrameKey(), it->landmarkKey,
        currentFrame.correspondenceDistributions.getKeyFromDataIndex(
            std::distance(currentFrame.correspondenceDistributions.begin(),
                          it))));
    ++nErrorTerms;
  }
  LOG_INFO("Created {} error terms for the current frame.", nErrorTerms);

  LOG_INFO(
      "Solving for current frame pose with fixed landmarks and host frames.");
  solver.settings.initialLambda = 1e3;
  auto result = solver.solveLevenbergMarquardt(variableContainer,
                                               errorTermContainer, prior);

  if (result.whitenedSqError.size()) {
    LOG_INFO(
        "Solved for current frame pose. Initial error: {} -> final error: {} "
        "with {} iterations.",
        result.whitenedSqError.front(), result.whitenedSqError.back(),
        result.whitenedSqError.size());
  }

  LOG_INFO("Updating current frame pose.");
  currentFrame.imustate.attitude = variableContainer.at(poseKey).value.so3();
  currentFrame.imustate.pos = variableContainer.at(poseKey).value.translation();
}
