#include "FrontFndVisualOdometry.h"

#include "spdlog/spdlog.h"

using namespace QDVO;

FrontEndVisualOdometry::FrontEndVisualOdometry()
{
    ArgMin::SE3 pose;
    poseKey = variableContainer.insert(std::move(pose));
    prior.addVariable(poseKey);
}

void FrontEndVisualOdometry::run(QDVO::Graph& graph)
{
    SPDLOG_INFO("Clearing old error terms.");
    // Ensure that the error term container is clear.
    errorTermContainer.clear();

    SPDLOG_INFO("Creating error terms for the current frame.");
    Frame& currentFrame = *graph.getCurrentFrame(); 
    int nErrorTerms = 0;
    for (auto it = currentFrame.correspondenceDistributions.begin(); it != currentFrame.correspondenceDistributions.end(); it++) {
        if (it->dormant) {
            continue;
        }
        auto landmark = graph.getLandmarkMap().at(it->landmarkKey);

        if (landmark->status != Landmark::LandmarkStatus::ACTIVE)
	{
		SPDLOG_WARN("Invalid landmark was in the correspondence distribution list {}", landmark->status);
		continue;
	}

        errorTermContainer.insert(QuasiDirectErrorTerm_TargetFrame(poseKey, Eigen::Matrix2d::Identity(), &graph, landmark->parentFrameKey, graph.getCurrentFrameKey(), it->landmarkKey, it - currentFrame.correspondenceDistributions.begin()));
        ++nErrorTerms;
    }
    SPDLOG_INFO("Created {} error terms for the current frame.", nErrorTerms);

    SPDLOG_INFO("Solving for current frame pose with fixed landmarks and host frames.");
    solver.settings.initialLambda = 1e3;
    auto result = solver.solveLevenbergMarquardt(variableContainer, errorTermContainer, prior);

    if (result.whitenedSqError.size()) {
        SPDLOG_INFO("Solved for current frame pose. Initial error: {} -> final error: {} with {} iterations.",  result.whitenedSqError.front(),  result.whitenedSqError.back(), result.whitenedSqError.size());
    }

    SPDLOG_INFO("Updating current frame pose.");
    currentFrame.imustate.attitude = variableContainer.at(poseKey).value.so3();
    currentFrame.imustate.pos = variableContainer.at(poseKey).value.translation();
}
