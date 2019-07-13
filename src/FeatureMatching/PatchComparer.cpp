#include "PatchComparer.h"

QDVO::PatchComparer::PatchComparer(){

}

SCALAR_TYPE QDVO::PatchComparer::compare(const Frame& targetFrame, const Eigen::Vector2i& pixel)
{
    cv::Rect roi(cv::Point2i(pixel(0) - PATCH_RADIUS, pixel(1) - PATCH_RADIUS), cv::Point2i(pixel(0) + PATCH_RADIUS, pixel(1) + PATCH_RADIUS));

}
