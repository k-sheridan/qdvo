#include "Graph.h"

#include "Landmark.h"
#include "Frame.h"
#include "CameraModel.hpp"

#include "Logging.h"

namespace QDVO {

Graph::Graph()
{
    currentFrameKey = keyframes.insert(std::make_unique<QDVO::Frame>());
    SPDLOG_INFO("Initialized current frame.");
}

void Graph::moveCurrentFrameIntoNewKeyframePosition()
{
    assert(keyframes.size() <= N_KEYFRAMES);

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

void Graph::moveCurrentFrameIntoMarginalizedKeyframePosition(const KeyframeMap::key_type marginalizedKeyframeKey)
{
    auto& currentFrame = getCurrentFrame();
    assert(currentFrame->status == Frame::FrameStatus::MARGINALIZED);

    auto& keyframe = *(keyframes.at(marginalizedKeyframeKey));

    assert(keyframe->status == Frame::FrameStatus::MARGINALIZED);

    // update the slot generation.
    auto newKey = keyframes.updateSlotGeneration(currentFrameKey);
    assert(!newKey.isInvalid());

    currentFrameKey = marginalizedKeyframeKey;
}

void Graph::moveCurrentFrameIntoKeyframePosition()
{
    if (keyframes.size() > N_KEYFRAMES)
    {
        //TODO find a marginalized keyframe to swap the current frame with.
        assert(false);
    }
    else
    {
        // there is enough room to make a new keyframe.
        this->moveCurrentFrameIntoNewKeyframePosition();
    }
}

std::vector<std::tuple<LandmarkMap::key_type, Vector2>> Graph::getVisibleLandmarksInCurrentFrame(bool activeLandmarksOnly, bool includeCurrentFrameLandmarks)
{
    SPDLOG_INFO("Computing visible landmarks in current frame.");
    std::vector<std::tuple<LandmarkMap::key_type, Vector2>> visibleLandmarkPtrs;

    auto& cm = cameraModelMap.at(getCurrentFrame()->cameraModelKey)->first;

    const SE3& T_cfimu_cfcam = *(extrinsics.at(getCurrentFrame()->extrinsicKey));

    // iterate through all keyframes and project their landmarks into the current frame
    for (auto &keyframe : keyframes)
    {
        if (keyframe->status != QDVO::Frame::FrameStatus::ACTIVE) {
            SPDLOG_INFO("skipping inactive or marginalized keyframe.");
            continue;
        }

        SPDLOG_INFO("computing visible landmarks for current frame.");

        const SE3& T_kfimu_kfcam = *(extrinsics.at(keyframe->extrinsicKey));

        // inv(T_w_cimu * T_imu_cam) * T_w_kimu * T_imu_cam
        SE3 T_cf_kf = (getCurrentFrame()->imustate.getSE3() * T_cfimu_cfcam).inverse() * (keyframe->imustate.getSE3() * T_kfimu_kfcam);

        for (auto &lKey : keyframe->landmarkKeys)
        {
            // Get the landmark.
            auto& l = *(landmarks.at(lKey));

            if ((l.status == Landmark::LandmarkStatus::ACTIVE || !activeLandmarksOnly) && l.status != Landmark::LandmarkStatus::MARGINALIZED)
            {
                // project landmarks
                auto px = cm->project(T_cf_kf * l.getEuclideanPoint());
                if (!px.has_value())
                {
                    SPDLOG_TRACE("failed to project landmark"); 
                    continue;
                }

                // add to vector

                visibleLandmarkPtrs.push_back(std::make_tuple(lKey, px.value()));
            }
        }
    }

    if (includeCurrentFrameLandmarks)
    {
        for (auto &lKey : getCurrentFrame()->landmarkKeys)
        {
            // Get the landmark.
            auto& l = *(landmarks.at(lKey));

            if ((l.status == Landmark::LandmarkStatus::ACTIVE || !activeLandmarksOnly) && l.status != Landmark::LandmarkStatus::MARGINALIZED)
            {
                visibleLandmarkPtrs.push_back(std::make_tuple(lKey, l.px));
            }
        }
    }

    return visibleLandmarkPtrs;
}

Vector3 Graph::projectLandmarkToCameraFrame(KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey, LandmarkMap::key_type landmarkKey)
{
    std::unique_ptr<Frame> &targetFrame = *(keyframes.at(targetFrameKey));
    std::unique_ptr<Frame> &sourceFrame = *(keyframes.at(sourceFrameKey));

    Landmark &l = *(landmarks.at(landmarkKey));

    assert(l.parentFrameKey == sourceFrameKey);
    
    return projectLandmarkToCameraFrame(*targetFrame, *sourceFrame, l);
}

Result<Vector2> Graph::projectLandmarkToPixel(KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey, LandmarkMap::key_type landmarkKey)
{
    std::unique_ptr<Frame> &targetFrame = *(keyframes.at(targetFrameKey));
    std::unique_ptr<Frame> &sourceFrame = *(keyframes.at(sourceFrameKey));

    Landmark &l = *(landmarks.at(landmarkKey));

    assert(l.parentFrameKey == sourceFrameKey);
    
    return projectLandmarkToPixel(*targetFrame, *sourceFrame, l);
}


/// Transforms the landmark into a euclidean point in the current frame.
QDVO::Vector3 Graph::projectLandmarkToCameraFrame(const Frame& targetFrame, const Frame& sourceFrame, const Landmark& landmark) 
{
    SE3 T_tf_sf = computeRelativeKeyframeTransform(targetFrame, sourceFrame); 
    return T_tf_sf * landmark.getEuclideanPoint();
}

/// Projects a landmark in the pixels in the current frame.
QDVO::Result<QDVO::Vector2> Graph::projectLandmarkToPixel(const Frame& targetFrame, const Frame& sourceFrame, const Landmark& landmark)
{
    auto pt = projectLandmarkToCameraFrame(targetFrame, sourceFrame, landmark);

    std::unique_ptr<CameraModel> &cm = cameraModelMap.at(targetFrame.cameraModelKey)->first;

    return cm->project(pt);
}

QDVO::SE3 Graph::computeRelativeKeyframeTransform(const Frame& targetFrame, const Frame& sourceFrame)
{
    SE3& T_tfimu_tfcam = *(extrinsics.at(targetFrame.extrinsicKey));
    SE3& T_sfimu_sfcam = *(extrinsics.at(sourceFrame.extrinsicKey));

    return (targetFrame.imustate.getSE3() * T_tfimu_tfcam).inverse() * (sourceFrame.imustate.getSE3() * T_sfimu_sfcam);
}

} // namespace QDVO
