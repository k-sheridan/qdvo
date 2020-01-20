#include "SlidingWindowEstimator.h"


using namespace QDVO;

SlidingWindowEstimator::SlidingWindowEstimator()
{
    
}


void SlidingWindowEstimator::run(QDVO::Graph& graph)
{
    //spdlog::set_level(spdlog::level::trace);
    // Check if a new keyframe has been added.
    for (auto it = graph.getKeyframeMap().begin(); it != graph.getKeyframeMap().end(); it++) {
        auto keyframeKey = graph.getKeyframeMap().getKeyFromDataIndex(it - graph.getKeyframeMap().begin());

        // Check if this keyframe is the current frame or it is not active.
        if (keyframeKey == graph.getCurrentFrameKey() || (*it)->status != Frame::FrameStatus::ACTIVE) {
            // Skip this frame.
            continue;
        }

        // Check if we have not added this frame before.
        bool keyframeNotInSWE = true;
        for (auto innerIt = poseKeyMap.begin(); innerIt != poseKeyMap.end(); innerIt++) {
            auto key = poseKeyMap.getKeyFromDataIndex(innerIt - poseKeyMap.begin());
            if (key == keyframeKey) {
                keyframeNotInSWE = false;
            }
        }
        // If the keyframe is new, add it and all its landmarks to the SWE.
        if (keyframeNotInSWE) {
            SPDLOG_INFO("Found new keyframe for sliding window estimator.");
            // Insert a variable for the keyframe pose.
            ArgMin::SE3 pose;
            pose.value = (*it)->imustate.getSE3();
            auto poseKey = variableContainer.insert(pose);
            // Insert a map from the variable key to keyframe key.
            poseKeyMap.insert(keyframeKey, poseKey);
            // Insert the variable into the prior.
            prior.addVariable(poseKey);

            // Insert the landmarks hosted by this keyframe into the estimator.
            for (auto landmarkKey : (*it)->landmarkKeys) {
                // Insert an inverse depth variable into the estimator.
                ArgMin::InverseDepth dinv;
                dinv.value = graph.getLandmarkMap().at(landmarkKey)->dinv;
                auto dinvKey = variableContainer.insert(dinv);
                // Insert mapping between variable and landmark key.
                dinvKeyMap.insert(landmarkKey, dinvKey);
                // Insert variable into the prior.
                prior.addVariable(dinvKey);
            }
            SPDLOG_INFO("Added new keyframe to estimator.");
            SPDLOG_INFO("Constructing error terms for new keyframe.");

            // Iterate through the correspondence distributions in this keyframe, and add them as error terms.
            int cdIdx = 0;
            int errorTermsForThisKeyframe = 0;
            for (auto& cd : (*it)->correspondenceDistributions) {
                if (cd.dormant) {
                    // Increment the correspondence distribution index.
                    ++cdIdx;
                    continue;
                }

                auto targetFramePoseVariableKey = *poseKeyMap.at(keyframeKey);
                assert(targetFramePoseVariableKey == poseKey);
                auto observedLandmarkIt = graph.getLandmarkMap().at(cd.landmarkKey);
                assert(observedLandmarkIt != graph.getLandmarkMap().end());
                auto sourceFramePoseVariableKey = *poseKeyMap.at(observedLandmarkIt->parentFrameKey);
                auto dinvVariableKey = *dinvKeyMap.at(cd.landmarkKey);

                QuasiDirectErrorTerm qdet(sourceFramePoseVariableKey, 
                                        targetFramePoseVariableKey, 
                                        dinvVariableKey, 
                                        Eigen::Matrix<double, 2, 2>::Identity(), 
                                        cdIdx,
                                        &graph, 
                                        observedLandmarkIt->parentFrameKey, 
                                        keyframeKey, 
                                        cd.landmarkKey);

                errorTermContainer.insert(qdet);

                // Increment the correspondence distribution index.
                ++errorTermsForThisKeyframe;
                ++cdIdx;
            }
            SPDLOG_INFO("Constructed {} error terms for the new keyframe", errorTermsForThisKeyframe);
        }
    }

    // Clean the prior before optimizing.
    prior.removeUnsedVariables(variableContainer);

    // Run the solver.
    auto result = solver.solveLevenbergMarquardt(variableContainer, errorTermContainer, prior);

    SPDLOG_INFO("Solver ran for {} iterations.", result.whitenedSqError.size());
    if (!result.whitenedSqError.empty()){
        SPDLOG_INFO("Initial squared error {} -> final squared error {}", result.whitenedSqError.front(), result.whitenedSqError.back());
    }

    // Apply the updates to the actual graph iff the error was decreased.
    if (!result.whitenedSqError.empty() && result.whitenedSqError.back() < result.whitenedSqError.front()){
        SPDLOG_INFO("SWE successfully reduced error, applying updates to graph.");
        synchronizeGraph(graph);
    } else {
        SPDLOG_ERROR("Error increased, not syncing update with graph.");
    }
}

void SlidingWindowEstimator::removeOutliers(QDVO::Graph& graph)
{
    // Iterate through all visual error terms and check if any residuals are too high.
    std::vector<LandmarkMap::key_type> landmarksToMarginalize; 
    auto& errorTermMap = errorTermContainer.getErrorTermMap<QDVO::QuasiDirectErrorTerm>();
    for (auto& errorTerm : errorTermMap) 
    {
	if (errorTerm.residual.norm() >= settings.pixelOutlierThreshold)
	{
	    landmarksToMarginalize.push_back(errorTerm.landmarkKey);
	}
    }

    // Marginalize all landmarks which are outliers.
    int marginalizedLandmarks = 0;
    for (auto& landmarkKey : landmarksToMarginalize)
    {
        if (graph.getLandmarkMap().at(landmarkKey)->status != Landmark::LandmarkStatus::MARGINALIZED)
	{
	    marginalizeLandmark(graph, landmarkKey);
	    ++marginalizedLandmarks;
	}
    }

    SPDLOG_INFO("Remove {} outlier landmarks.", marginalizedLandmarks);
}

void SlidingWindowEstimator::runMarginalizationStrategy(QDVO::Graph& graph)
{

}

void SlidingWindowEstimator::synchronizeGraph(QDVO::Graph& graph)
{
    for (auto it = poseKeyMap.begin(); it != poseKeyMap.end(); it++)
    {
        auto key = poseKeyMap.getKeyFromDataIndex(it - poseKeyMap.begin());
        auto& keyframe = (*graph.getKeyframeMap().at(key));
        auto& variable = variableContainer.at(*it);
        keyframe->imustate.pos = variable.value.translation();
        keyframe->imustate.attitude = variable.value.so3();
    }

    for (auto it = dinvKeyMap.begin(); it != dinvKeyMap.end(); it++)
    {
        auto key = dinvKeyMap.getKeyFromDataIndex(it - dinvKeyMap.begin());
        auto& landmark = (*graph.getLandmarkMap().at(key));
        auto& variable = variableContainer.at(*it);
        landmark.dinv = variable.value;
    }
}

void SlidingWindowEstimator::marginalizeLandmark(QDVO::Graph& graph, LandmarkMap::key_type landmarkKey)
{
    auto variableKey = *dinvKeyMap.at(landmarkKey);
    // First marginalize the landmark.
    marginalizer.marginalizeVariable(variableKey, prior, errorTermContainer, ArgMin::VariableGroup<>(), settings.pixelOutlierThreshold);
    // Delete the landmark variable from the SWE.
    variableContainer.erase(variableKey);
    dinvKeyMap.erase(landmarkKey);
    // Set the landmark to marginalized.
    graph.getLandmarkMap().at(landmarkKey)->status = Landmark::LandmarkStatus::MARGINALIZED;
}

void SlidingWindowEstimator::marginalizeKeyframe(QDVO::Graph& graph, KeyframeMap::key_type keyframeKey)
{
    // First marginalize all landmarks one at a time.
    auto& keyframe = *(*graph.getKeyframeMap().at(keyframeKey));
    for (auto& landmarkKey : keyframe.landmarkKeys)
    {
	marginalizeLandmark(graph, landmarkKey);
    }

    // Finally marginalize the keyframe itself while ignoring landmark correlations to preserve sparsity.
    auto variableKey = *poseKeyMap.at(keyframeKey);
    marginalizer.marginalizeVariable(variableKey, prior, errorTermContainer, ArgMin::VariableGroup<ArgMin::InverseDepth>(), settings.pixelOutlierThreshold);
    variableContainer.erase(variableKey);
    poseKeyMap.erase(keyframeKey);
    // Set the keyframe to marginalized.
    keyframe.status = Frame::FrameStatus::MARGINALIZED;
}
