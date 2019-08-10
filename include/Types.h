#pragma once

#include "GlobalDefinitions.h"
#include <sophus/se3.hpp>
#include <optional>

namespace QDVO
{

using SE3 = Sophus::SE3<SCALAR_TYPE>;
using SO3 = Sophus::SO3<SCALAR_TYPE>;
using Vector3 = Eigen::Matrix<SCALAR_TYPE, 3, 1>;
using Vector2 = Eigen::Matrix<SCALAR_TYPE, 2, 1>;

template<typename T>
using Result = std::optional<T>;

} // namespace QDVO
