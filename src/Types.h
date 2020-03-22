#pragma once

#include "GlobalDefinitions.h"
#include <sophus/se3.hpp>
#include "Optimizer/SlotMap.h"
#include <memory>
#include <optional>

namespace QDVO
{

using Scalar = SCALAR_TYPE;
using SE3 = Sophus::SE3<Scalar>;
using SO3 = Sophus::SO3<Scalar>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;

template<typename T>
using Result = std::optional<T>;

class Frame;
class CameraModel;
class Landmark;
class CorrespondenceDistribution;

using KeyframeMap = ArgMin::SlotMap<std::unique_ptr<Frame>>;
using CameraModelMap = ArgMin::SlotMap<std::pair<std::unique_ptr<CameraModel>, QDVO::SE3>>;
using LandmarkMap = ArgMin::SlotMap<Landmark>;
using ExtrinsicMap = ArgMin::SlotMap<QDVO::SE3>;
using CorrespondenceDistributionMap = ArgMin::SlotMap<QDVO::CorrespondenceDistribution>;

} // namespace QDVO
