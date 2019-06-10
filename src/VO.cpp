#include "VO.h"

QDVO::VO::VO()
{

}

void QDVO::VO::setCameraModel(std::unique_ptr<QDVO::CameraModel>& cameraModelPtr, const ID_TYPE cameraID)
{
    this->graph.cameraModelMap.insert(std::pair<ID_TYPE, std::unique_ptr<CameraModel> >(cameraID, std::unique_ptr<QDVO::CameraModel>()));

    // give ownership to the unique pointer in the table.
    this->graph.cameraModelMap.at(cameraID).swap(cameraModelPtr);
}
