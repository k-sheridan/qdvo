#pragma once

#include "Frame.h"
#include <opencv2/core.hpp>
#include <unordered_map>
#include <memory>
#include "GlobalDefinitions.h"
#include "Settings.h"
#include "CameraModel.hpp"
#include "FeatureDetector.h"

namespace  QDVO {

typedef std::unordered_map<ID_TYPE, std::unique_ptr<Frame> > KeyframeSetType;
typedef std::unordered_map<ID_TYPE, std::unique_ptr<CameraModel> > CameraModelMapType;

class Graph
{
public:
    Graph();

    // Sets the camera model for the given cam ID. NOTE: QDVO creates its own local copy.
    void setCameraModel(std::unique_ptr<QDVO::CameraModel>& cameraModelPtr, const ID_TYPE cameraID)
    {
        this->cameraModelMap.insert(std::pair<ID_TYPE, std::unique_ptr<CameraModel> >(cameraID, std::unique_ptr<QDVO::CameraModel>()));
        // give ownership to the unique pointer in the table.
        this->cameraModelMap.at(cameraID).swap(cameraModelPtr);
    }

    Frame& getCurrentFrame();

    KeyframeSetType& getKeyframeSet(){return this->keyframeSet;}

    /*
     * Looks across the keyframe set and current frame for the highest frame ID and returns one id higher
     */
    ID_TYPE getNewFrameID();

    /*
     * This function will swap the current frame and the marginalized keyframe and update the hash table key to reflect the new keyframe id.
     * The current frame is now equal to the marginalized keyframe. For safety, you should always check that a frame is not marginalized when using it.
     */
    void moveCurrentFrameIntoMarginalizedKeyframePosition(const ID_TYPE marginalizedKeyframeID);

private:
    // Members, nodes, and edges of the graph.

    CameraModelMapType cameraModelMap; // camID to camera model mapping. Done this way for memory/compute efficiency.

    KeyframeSetType keyframeSet; // gives mapping from keyframe ids to keyframes. bounds the memory consumption.

    std::unique_ptr<Frame> currentFrame;
};
}

