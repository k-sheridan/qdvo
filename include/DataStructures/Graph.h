#pragma once

#include <Frame.h>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <memory>
#include <GlobalDefinitions.h>
#include <Settings.h>
#include <CameraModel.hpp>
#include <FeatureDetector.h>

namespace  QDVO {
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

    // Members, nodes, and edges of the graph.

    std::unordered_map<ID_TYPE, std::unique_ptr<CameraModel>> cameraModelMap; // camID to camera model mapping. Done this way for memory/compute efficiency.

    std::unordered_map<ID_TYPE, std::unique_ptr<Frame>> keyframeSet; // gives mapping from keyframe ids to keyframes. bounds the memory consumption.

    std::unique_ptr<Frame> currentFrame; // A preallocated frame for the current frame to reside in.
};
}

