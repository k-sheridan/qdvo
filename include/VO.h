#pragma once

#include <Frame.h>
#include <opencv4/opencv2/core.hpp>
#include <map>
#include <GlobalDefinitions.h>
#include <Settings.h>
#include <CameraModel.hpp>
#include <FeatureDetector.h>
#include <Graph.h>

namespace QDVO {
class VO
{
public:
    VO();

    void setCameraModel(std::unique_ptr<QDVO::CameraModel>& cameraModelPtr, const ID_TYPE cameraID); // Sets the camera model for the given cam ID. NOTE: QDVO creates its own local copy.


    //MEMBERS

    Graph graph; // stores the state of the VO system. Used as argument to the optimizers.

    std::unique_ptr<QDVO::FeatureDetector> featureDetector; // pointer to a feature detector implementation
};
}

