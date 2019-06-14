#include "Graph.h"

QDVO::Graph::Graph()
{

}

QDVO::Frame& QDVO::Graph::getCurrentFrame()
{
    return *(this->currentFrame.get());
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

    assert(highestFrameID > 0);

    return highestFrameID;
}
