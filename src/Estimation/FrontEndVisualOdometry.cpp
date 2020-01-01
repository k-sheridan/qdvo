#include "FrontFndVisualOdometry.h"

using namespace QDVO;

FrontEndVisualOdometry::FrontEndVisualOdometry()
{
    ArgMin::SE3 pose;
    poseKey = variableContainer.insert(std::move(pose));
}

void FrontEndVisualOdometry::run(QDVO::Graph& graph)
{

}
