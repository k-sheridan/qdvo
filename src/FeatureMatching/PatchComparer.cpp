#include "PatchComparer.h"

QDVO::PatchComparer::PatchComparer(){

}

QDVO::ResultType<SCALAR_TYPE> QDVO::PatchComparer::compare(QDVO::Patch& patch, Frame& targetFrame, Eigen::Vector2i& pixel)
{
    cv::Rect roi(cv::Point2i(pixel(0) - PATCH_RADIUS, pixel(1) - PATCH_RADIUS), cv::Size2i(PATCH_WIDTH, PATCH_WIDTH));

    assert(this->meanStdDevTable.framePtr == &targetFrame);
    assert(patch.getStdDev() > 1e-8);
    assert(PATCH_WIDTH == 2*PATCH_RADIUS + 1);

    cv::Mat& targetImage = targetFrame.imagePyr.getImage();

    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width >= targetImage.cols || roi.y + roi.height >= targetImage.rows)
    {
        return QDVO::ResultType<SCALAR_TYPE>();
    }

    // Get the approximate mean and standard deviation of the test patch
    float approxMean = this->meanStdDevTable.getMean(cv::Point2i(pixel(0), pixel(1)));
    float approxStdDev = this->meanStdDevTable.getStdDev(cv::Point2i(pixel(0), pixel(1)));

    // get the test patch from the image and compute its zero mean self.
    cv::Mat testData = targetImage(roi);
    QDVO::Patch testPatch = QDVO::Patch(testData, approxMean, approxStdDev);
   
    SCALAR_TYPE resultingScore = testPatch.getZeroMeanImageMatrix()(5, 5);

    return QDVO::ResultType<SCALAR_TYPE>(resultingScore);
}
