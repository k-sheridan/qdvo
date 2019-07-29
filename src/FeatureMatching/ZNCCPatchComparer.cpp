#include "PatchComparer.h"

QDVO::ZNCCPatchComparer::ZNCCPatchComparer(){

}

QDVO::ResultType<SCALAR_TYPE> QDVO::ZNCCPatchComparer::compare(QDVO::Patch& patch, Frame& targetFrame, Eigen::Vector2i& pixel)
{
    cv::Rect roi(cv::Point2i(pixel(0) - PATCH_RADIUS, pixel(1) - PATCH_RADIUS), patch.getImageData().size());

    assert(patch.getImageData().size().height == patch.getImageData().size().width && patch.getImageData().size().width == PATCH_RADIUS*2 + 1);
    assert(this->meanStdDevTable.framePtr == &targetFrame);
    assert(patch.getStdDev() > 1e-8);

    cv::Mat& targetImage = targetFrame.imagePyr.getImage();

    if (roi.x < 0 || roi.y < 0 || roi.x + roi.width >= targetImage.cols || roi.y + roi.height >= targetImage.rows)
    {
        return QDVO::ResultType<SCALAR_TYPE>();
    }

    // Get the approximate mean and standard deviation of the test patch
    //float approxMean = this->meanStdDevTable.getMean(cv::Point2i(pixel(0), pixel(1)));
    //float approxStdDev = this->meanStdDevTable.getStdDev(cv::Point2i(pixel(0), pixel(1)));
    cv::Mat rawData = targetImage(roi);
    QDVO::Patch tp = QDVO::Patch(rawData);

    if (tp.getStdDev() <= 1e-8)
    {
        return QDVO::ResultType<SCALAR_TYPE>();
    }

    // Compute the zero mean test patch
    cv::Mat correlationMat;

    cv::multiply(tp.getZeroMeanImageData(), patch.getZeroMeanImageData(), correlationMat);

    cv::Scalar sumCorrelation = cv::sum(correlationMat);

    SCALAR_TYPE resultingScore = sumCorrelation[0] / (tp.getStdDev() * patch.getStdDev() * patch.getImageData().size().width * patch.getImageData().size().width);

    return QDVO::ResultType<SCALAR_TYPE>(resultingScore);
}
