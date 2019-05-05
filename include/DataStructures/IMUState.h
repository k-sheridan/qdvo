#pragma once

#include <sophus/common.hpp>
#include <sophus/types.hpp>
#include <sophus/se3.hpp>
#include <GlobalDefinitions.h>

class IMUState
{
public:
    IMUState();

    Sophus::SE3<SCALAR_TYPE> pose;
};

