#include "PatchComparer.h"

QDVO::PatchComparer::PatchComparer(){

}

bool QDVO::PatchComparer::compare(SCALAR_TYPE& resultingScore, const QDVO::Patch& patch, const Frame& targetFrame, const Eigen::Vector2i& pixel)
{
    cv::Rect roi(cv::Point2i(pixel(0) - PATCH_RADIUS, pixel(1) - PATCH_RADIUS), cv::Point2i(pixel(0) + PATCH_RADIUS, pixel(1) + PATCH_RADIUS));

    assert(this->meanStdDevTable.framePtr == &targetFrame);

    return false;
}
