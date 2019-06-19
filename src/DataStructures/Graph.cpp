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

std::unique_ptr<QDVO::Frame>& QDVO::Graph::getKeyframe(const ID_TYPE keyframeID)
{
    return (this->keyframeSet.at(keyframeID));
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
