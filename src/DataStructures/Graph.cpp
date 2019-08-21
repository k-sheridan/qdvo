#include "Graph.h"

QDVO::Graph::Graph()
{
}

std::unique_ptr<QDVO::Frame> &QDVO::Graph::getCurrentFrame()
{
    return (this->currentFrame);
}

std::unique_ptr<QDVO::CameraModel> &QDVO::Graph::getCameraModel(const ID_TYPE cameraID)
{
    return (this->cameraModelMap.at(cameraID));
}

QDVO::SE3 QDVO::Graph::getExtrinsic(const ID_TYPE cameraID)
{
    assert(this->extrinsicSet.count(cameraID));
    return this->extrinsicSet.at(cameraID);
}

std::unique_ptr<QDVO::Frame> &QDVO::Graph::getKeyframe(const ID_TYPE keyframeID)
{
    return (this->keyframeSet.at(keyframeID));
}

std::unique_ptr<QDVO::Frame> &QDVO::Graph::getFrame(const ID_TYPE frameID)
{
    if (this->currentFrame->frameID == frameID)
    {
        return this->currentFrame;
    }
    else
    {
        return this->getKeyframe(frameID);
    }
}

void QDVO::Graph::moveCurrentFrameIntoNewKeyframePosition()
{
    assert(this->currentFrame->status == QDVO::Frame::FrameStatus::ACTIVE);
    assert(this->keyframeSet.size() <= N_KEYFRAMES);

    // make room for another keyframe
    this->keyframeSet.insert({this->currentFrame->frameID, std::unique_ptr<QDVO::Frame>(new QDVO::Frame())});

    // copy over vital information
    QDVO::Frame &tmpFrame = *(this->keyframeSet.at(this->currentFrame->frameID));
    tmpFrame.imustate = this->currentFrame->imustate;
    tmpFrame.camID = this->currentFrame->camID;
    tmpFrame.frameID = 0;

    // swap the current frame into its new spot.
    this->keyframeSet.at(this->currentFrame->frameID).swap(this->currentFrame);
}

void QDVO::Graph::moveCurrentFrameIntoMarginalizedKeyframePosition(const ID_TYPE marginalizedKeyframeID)
{
    assert(this->keyframeSet.at(marginalizedKeyframeID)->status == QDVO::Frame::FrameStatus::MARGINALIZED);
    assert(this->currentFrame->status == QDVO::Frame::FrameStatus::MARGINALIZED);

    auto nh = this->keyframeSet.extract(marginalizedKeyframeID);

    nh.key() = this->currentFrame->frameID;
    nh.mapped().swap(this->currentFrame);

    this->keyframeSet.insert(std::move(nh)); // insert the updated key-value pair
}

void QDVO::Graph::moveCurrentFrameIntoKeyframePosition()
{
    if (this->keyframeSet.size() > N_KEYFRAMES)
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

ID_TYPE QDVO::Graph::getNewFrameID()
{
    ID_TYPE highestFrameID = 0;
    for (auto &e : this->keyframeSet)
    {
        assert(e.first == e.second->frameID);
        if (e.second->frameID > highestFrameID)
        {
            highestFrameID = e.second->frameID;
        }
    }

    if (this->currentFrame->frameID > highestFrameID)
    {
        highestFrameID = this->currentFrame->frameID;
    }

    assert(highestFrameID + 1 > 0);

    return highestFrameID + 1;
}

std::vector<std::tuple<QDVO::Landmark *, QDVO::Vector2>> QDVO::Graph::getVisibleLandmarksInCurrentFrame(bool activeLandmarksOnly, bool includeCurrentFrameLandmarks)
{
    std::vector<std::tuple<QDVO::Landmark *, QDVO::Vector2>> visibleLandmarkPtrs;

    std::unique_ptr<QDVO::Frame> &currentFrame = this->getCurrentFrame();
    std::unique_ptr<QDVO::CameraModel> &cm = this->getCameraModel(currentFrame->camID);

    QDVO::SE3 T_cfimu_cfcam = this->getExtrinsic(currentFrame->camID);

    // iterate through all keyframes and project their landmarks into the current frame
    for (auto &element : this->keyframeSet)
    {
        std::cout << "computing visible landmarks for kf " << element.first << std::endl;

        QDVO::SE3 T_kfimu_kfcam = this->getExtrinsic(element.second->camID);

        // inv(T_w_cimu * T_imu_cam) * T_w_kimu * T_imu_cam
        QDVO::SE3 T_cf_kf = (currentFrame->imustate.getSE3() * T_cfimu_cfcam).inverse() * (element.second->imustate.getSE3() * T_kfimu_kfcam);

        for (auto &l : element.second->landmarks)
        {

            if ((l.status == QDVO::Landmark::LandmarkStatus::ACTIVE || !activeLandmarksOnly) && l.status != QDVO::Landmark::LandmarkStatus::MARGINALIZED)
            {
                // project landmarks
                auto px = cm->project(T_cf_kf * l.getEuclideanPoint());
                if (!px.has_value())
                {
                    std::cout << "failed to project: " << l.getEuclideanPoint() << " -> " << T_cf_kf * l.getEuclideanPoint() << std::endl;
                    continue;
                }

                // add to vector

                visibleLandmarkPtrs.push_back(std::make_tuple(&l, px.value()));
            }
        }
    }

    if (includeCurrentFrameLandmarks)
    {
        for (auto &l : currentFrame->landmarks)
        {
            if ((l.status == QDVO::Landmark::LandmarkStatus::ACTIVE || !activeLandmarksOnly) && l.status != QDVO::Landmark::LandmarkStatus::MARGINALIZED)
            {
                visibleLandmarkPtrs.push_back(std::make_tuple(&l, l.px));
            }
        }
    }

    return visibleLandmarkPtrs;
}

QDVO::Vector3 QDVO::Graph::projectLandmarkToCameraFrame(ID_TYPE targetFrameID, ID_TYPE sourceFrameID, ID_TYPE landmarkID)
{
    std::unique_ptr<QDVO::Frame> &targetFrame = this->getFrame(targetFrameID);
    std::unique_ptr<QDVO::Frame> &sourceFrame = this->getFrame(sourceFrameID);

    QDVO::Landmark &l = sourceFrame->landmarks.at(landmarkID - 1);
    assert(l.landmarkID == landmarkID);
    assert(l.parentFrameID == sourceFrameID);

    QDVO::SE3 T_tfimu_tfcam = this->getExtrinsic(targetFrame->camID);
    QDVO::SE3 T_sfimu_sfcam = this->getExtrinsic(sourceFrame->camID);

    QDVO::SE3 T_tf_sf = (targetFrame->imustate.getSE3() * T_tfimu_tfcam).inverse() * (sourceFrame->imustate.getSE3() * T_sfimu_sfcam);

    return T_tf_sf * l.getEuclideanPoint();
}

QDVO::Result<QDVO::Vector2> QDVO::Graph::projectLandmarkToPixel(ID_TYPE targetFrameID, ID_TYPE sourceFrameID, ID_TYPE landmarkID)
{
    QDVO::Vector3 pt = this->projectLandmarkToCameraFrame(targetFrameID, sourceFrameID, landmarkID);

    std::unique_ptr<QDVO::CameraModel> &cm = this->getCameraModel(this->getFrame(targetFrameID)->camID);

    return cm->project(pt);
}
