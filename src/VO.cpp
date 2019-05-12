#include "VO.h"

QDVO::VO::VO()
{

}

template<class CameraModelType>
void QDVO::VO::setCameraModel(const CameraModelType& cameraModel, const ID_TYPE cameraID)
{
    this->cameraModelMap.insert(std::pair<ID_TYPE, std::unique_ptr<CameraModel> >(cameraID, std::unique_ptr<CameraModel>(cameraModel)));
}
