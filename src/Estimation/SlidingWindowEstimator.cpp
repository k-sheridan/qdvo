#include "SlidingWindowEstimator.h"

#include "spdlog/spdlog.h"

using namespace QDVO;

SlidingWindowEstimator::SlidingWindowEstimator()
{
    
}


void SlidingWindowEstimator::run(QDVO::Graph& graph)
{
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

    // Run the solver.
    auto result = solver.solveLevenbergMarquardt(variableContainer, errorTermContainer, prior);
    SPDLOG_INFO("Solver ran for {} iterations.", result.whitenedSqError.size());
    if (!result.whitenedSqError.empty())
        SPDLOG_INFO("Initial squared error {} -> final squared error {}", result.whitenedSqError.front(), result.whitenedSqError.back());
}

void SlidingWindowEstimator::removeOutliers(QDVO::Graph& graph)
{

}

void SlidingWindowEstimator::runMarginalizationStrategy(QDVO::Graph& graph)
{

}
