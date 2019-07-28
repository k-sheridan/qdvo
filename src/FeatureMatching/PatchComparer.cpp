#include "PatchComparer.h"

QDVO::PatchComparer::PatchComparer(){

}

bool QDVO::PatchComparer::compare(SCALAR_TYPE& resultingScore, QDVO::Patch& patch, Frame& targetFrame, Eigen::Vector2i& pixel)
{
    cv::Rect roi(cv::Point2i(pixel(0) - PATCH_RADIUS, pixel(1) - PATCH_RADIUS), patch.getImageData().size());

    assert(patch.getImageData().size().height == patch.getImageData().size().width && patch.getImageData().size().width == PATCH_RADIUS*2 + 1);
    assert(this->meanStdDevTable.framePtr == &targetFrame);
    assert(patch.getStdDev() > 1e-8);

    cv::Mat& targetImage = targetFrame.imagePyr.getImage();

    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width >= targetImage.cols || roi.y + roi.height >= targetImage.rows)
    {
        return false;
    }

    // Get the approximate mean and standard deviation of the test patch
    float approxMean = this->meanStdDevTable.getMean(cv::Point2i(pixel(0), pixel(1)));
    float approxStdDev = this->meanStdDevTable.getStdDev(cv::Point2i(pixel(0), pixel(1)));

    if (approxStdDev <= 1e-8)
    {
        return false;
    }

    // Compute the zero mean test patch
    cv::Mat zeroMeanTestPatch, correlationMat;
    cv::subtract(targetImage(roi), cv::Scalar(approxMean), zeroMeanTestPatch, cv::noArray(), CV_32S);

    cv::multiply(zeroMeanTestPatch, patch.getZeroMeanImageData(), correlationMat);

    cv::Scalar sumCorrelation = cv::sum(correlationMat);

    resultingScore = sumCorrelation[0] / (approxStdDev * patch.getStdDev() * patch.getImageData().size().width * patch.getImageData().size().width);

    return true;
}
