#include "Graph.h"

#include "Landmark.h"
#include "Frame.h"
#include "CameraModel.hpp"

namespace QDVO {

Graph::Graph()
{
}

void Graph::moveCurrentFrameIntoNewKeyframePosition()
{
    assert(currentFrame->status == Frame::FrameStatus::ACTIVE);
    assert(keyframes.size() <= N_KEYFRAMES);

    // make room for another keyframe
    auto newFrame = std::make_unique<Frame>();
    auto newKeyframeKey = keyframes.insert(std::move(newFrame));

    // copy over vital information
    std::unique_ptr<Frame>& newKeyframe = *(keyframes.at(newKeyframeKey));
    newKeyframe->imustate = currentFrame->imustate;
    newKeyframe->cameraModelKey = currentFrame->cameraModelKey;

    // swap the current frame into its new spot.
    newKeyframe.swap(currentFrame);
}

void Graph::moveCurrentFrameIntoMarginalizedKeyframePosition(const KeyframeMap::key_type marginalizedKeyframeKey)
{
    assert(currentFrame->status == Frame::FrameStatus::MARGINALIZED);

    auto& keyframe = *(keyframes.at(marginalizedKeyframeKey));

    assert(keyframe->status == Frame::FrameStatus::MARGINALIZED);

    keyframe.swap(currentFrame);

    // update the slot generation.
    auto newKey = keyframes.updateSlotGeneration(marginalizedKeyframeKey);
    assert(!newKey.isInvalid());
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
    std::vector<std::tuple<LandmarkMap::key_type, Vector2>> visibleLandmarkPtrs;

    auto& cm = cameraModelMap.at(currentFrame->cameraModelKey)->first;

    const SE3& T_cfimu_cfcam = *(extrinsics.at(currentFrame->extrinsicKey));

    // iterate through all keyframes and project their landmarks into the current frame
    for (auto &keyframe : keyframes)
    {
        std::cout << "computing visible landmarks for kf " << std::endl;

        const SE3& T_kfimu_kfcam = *(extrinsics.at(keyframe->extrinsicKey));

        // inv(T_w_cimu * T_imu_cam) * T_w_kimu * T_imu_cam
        SE3 T_cf_kf = (currentFrame->imustate.getSE3() * T_cfimu_cfcam).inverse() * (keyframe->imustate.getSE3() * T_kfimu_kfcam);

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
                    std::cout << "failed to project: " << l.getEuclideanPoint() << " -> " << T_cf_kf * l.getEuclideanPoint() << std::endl;
                    continue;
                }

                // add to vector

                visibleLandmarkPtrs.push_back(std::make_tuple(lKey, px.value()));
            }
        }
    }

    if (includeCurrentFrameLandmarks)
    {
        for (auto &lKey : currentFrame->landmarkKeys)
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

    SE3& T_tfimu_tfcam = *(extrinsics.at(targetFrame->extrinsicKey));
    SE3& T_sfimu_sfcam = *(extrinsics.at(sourceFrame->extrinsicKey));

    SE3 T_tf_sf = (targetFrame->imustate.getSE3() * T_tfimu_tfcam).inverse() * (sourceFrame->imustate.getSE3() * T_sfimu_sfcam);

    return T_tf_sf * l.getEuclideanPoint();
}

Result<Vector2> Graph::projectLandmarkToPixel(KeyframeMap::key_type targetFrameKey, KeyframeMap::key_type sourceFrameKey, LandmarkMap::key_type landmarkKey)
{
    Vector3 pt = this->projectLandmarkToCameraFrame(targetFrameKey, sourceFrameKey, landmarkKey);

    std::unique_ptr<CameraModel> &cm = cameraModelMap.at((*keyframes.at(targetFrameKey))->cameraModelKey)->first;

    return cm->project(pt);
}

} // namespace QDVO