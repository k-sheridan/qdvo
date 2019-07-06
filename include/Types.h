#pragma once

#include "GlobalDefinitions.h"
#include <sophus/se3.hpp>

namespace QDVO {

typedef Sophus::SE3<SCALAR_TYPE> SE3;
typedef Sophus::SO3<SCALAR_TYPE> SO3;
typedef Eigen::Matrix<SCALAR_TYPE, 3, 1> Vector3;
typedef Eigen::Matrix<SCALAR_TYPE, 2, 1> Vector2;

}
