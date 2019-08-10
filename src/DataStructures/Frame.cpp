#include "Frame.h"

QDVO::Frame::Frame()
{
    this->status = FrameStatus::INACTIVE;
    this->imagePyr = QDVO::ImagePyramid(IMAGE_PYRAMID_LEVELS);
    this->frameID = 0;
    this->camID = 0;
}

void QDVO::Frame::updateImage(cv::Mat &baseImage)
{
    // set the max intensity
    std::cout << baseImage.type() << std::endl;
    switch (baseImage.type())
    {
    case CV_8U:
        this->maxImageIntensity = 255;
        break;

    case CV_16U:
        this->maxImageIntensity = 65535;
        break;

    // Just add the maximum value for the type.
    default:
        throw std::runtime_error("image type not supported.");
    }

    this->imagePyr.generate(baseImage);
}

void QDVO::Frame::reset()
{
    // remove the landmarks
    this->landmarks.clear();

    // reset all correspondence distributions
    this->resetCorrespondenceDistributions();
}
