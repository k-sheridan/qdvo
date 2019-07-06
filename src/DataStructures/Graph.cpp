#include "Graph.h"

QDVO::Graph::Graph()
{

}

std::unique_ptr<QDVO::Frame>& QDVO::Graph::getCurrentFrame()
{
    return (this->currentFrame);
}

std::unique_ptr<QDVO::CameraModel>& QDVO::Graph::getCameraModel(const ID_TYPE cameraID)
{
    return (this->cameraModelMap.at(cameraID));
}

QDVO::SE3 QDVO::Graph::getExtrinsic(const ID_TYPE cameraID)
{
    return this->extrinsicSet.at(cameraID);
}

std::unique_ptr<QDVO::Frame>& QDVO::Graph::getKeyframe(const ID_TYPE keyframeID)
{
    return (this->keyframeSet.at(keyframeID));
}

std::unique_ptr<QDVO::Frame>& QDVO::Graph::getFrame(const ID_TYPE frameID)
{
    if (this->currentFrame->frameID == frameID)
    {
        return this->currentFrame;
    }
    else {
        return this->getKeyframe(frameID);
    }
}

void QDVO::Graph::moveCurrentFrameIntoNewKeyframePosition()
{
    assert(this->currentFrame->status == QDVO::Frame::FrameStatus::ACTIVE);
    assert(this->keyframeSet.size() <= N_KEYFRAMES);

    // make room for another keyframe
    this->keyframeSet.insert({this->currentFrame->frameID, std::unique_ptr<QDVO::Frame>()});

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

ID_TYPE QDVO::Graph::getNewFrameID()
{
    ID_TYPE highestFrameID = 0;
    for (auto& e : this->keyframeSet)
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

    //assert(highestFrameID > 0);

    return highestFrameID + 1;
}


std::vector<QDVO::Landmark*> QDVO::Graph::getVisibleLandmarksInCurrentFrame(bool activeLandmarksOnly)
{
    std::vector<QDVO::Landmark*> visibleLandmarkPtrs;

    std::unique_ptr<QDVO::Frame>& currentFrame = this->getCurrentFrame();
    std::unique_ptr<QDVO::CameraModel>& cm = this->getCameraModel(currentFrame->camID);

    QDVO::SE3 T_cfimu_cfcam = this->getExtrinsic(currentFrame->camID);

    // iterate through all keyframes and project their landmarks into the current frame
    for (auto& element : this->keyframeSet){
        std::cout << "computing visible landmarks for kf " << element.first << std::endl;

        QDVO::SE3 T_kfimu_kfcam = this->getExtrinsic(element.second->camID);

        // inv(T_w_cimu * T_imu_cam) * T_w_kimu * T_imu_cam
        QDVO::SE3 T_cf_kf = (currentFrame->imustate.getSE3() * T_cfimu_cfcam).inverse() * (element.second->imustate.getSE3() * T_kfimu_kfcam);

        for (auto& l : element.second->landmarks)
        {

            if (l.status == QDVO::Landmark::LandmarkStatus::ACTIVE || !activeLandmarksOnly)
            {
                // project landmarks
                try {
                    QDVO::Vector2 px = cm->project(T_cf_kf * l.getEuclideanPoint());
                } catch (const std::runtime_error& e) {
                    continue;
                }
                // add to vector
                visibleLandmarkPtrs.push_back(&l);
            }

        }
    }


    return visibleLandmarkPtrs;
}

QDVO::Vector3 QDVO::Graph::projectLandmarkToCameraFrame(ID_TYPE targetFrameID, ID_TYPE sourceFrameID, ID_TYPE landmarkID)
{
    std::unique_ptr<QDVO::Frame>& targetFrame = this->getFrame(targetFrameID);
    std::unique_ptr<QDVO::Frame>& sourceFrame = this->getFrame(sourceFrameID);

    QDVO::Landmark& l = sourceFrame->landmarks.at(landmarkID);
    assert(l.landmarkID == landmarkID);
    assert(l.parentFrameID == sourceFrameID);

    QDVO::SE3 T_tfimu_tfcam = this->getExtrinsic(targetFrame->camID);
    QDVO::SE3 T_sfimu_sfcam = this->getExtrinsic(sourceFrame->camID);

    QDVO::SE3 T_tf_sf = (targetFrame->imustate.getSE3() * T_tfimu_tfcam).inverse() * (sourceFrame->imustate.getSE3() * T_sfimu_sfcam);

    return T_tf_sf * l.getEuclideanPoint();
}
