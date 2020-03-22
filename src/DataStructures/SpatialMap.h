#pragma once

#include <array>

#include "Optimizer/SlotArray.h"
#include "Optimizer/SlotMap.h"

#include <Eigen/Core>

namespace QDVO {

/**
 * SpatialMap is a memory efficient datastructure for representing a 2D array of
 * information. The motivation behind this datastructure is a use case where
 * only a subset of a 2D array contains information.
 *
 * In QDVO, this is used by the CorrespondenceDistribution for fast memory
 * efficient searches in pixel space.
 *
 * SpatialMap is based on a SlotArray which allows the SpatialMap to resuse the
 * memory used to store information
 *
 * @tparam T stored information type. This type must contain a member function,
 * reset.
 * @tparam BucketWidth The width of the data bucket storing the information.
 */
template <typename T, int BucketWidth = 32, int MaxImageWidth = 1024>
class SpatialMap {
 public:
  SpatialMap() {}

  SpatialMap(const unsigned width) {}

  /// Key as defined in SlotMap
  struct BucketKey;

  /// Inherit from the slot array to add some
  using MapContainer =
      ArgMin::SlotArray<std::array<T, BucketWidth * BucketWidth>,
                        ArgMin::TypedSlotMapKey<BucketKey>>;

  void reset() {
    for (auto& e : data) {
      for (auto& pc : e) {
        pc.reset();
      }
    }
    data.clear();
  }

  T& get(const Eigen::Vector2i& pixel) {
    ArgMin::TypedSlotMapKey<BucketKey> handle = {topHash(pixel(0), pixel(1)),
                                                 0};

    // Get of insert a bucket in the map.
    auto& bucket = data.insert(handle);

    return bucket[bottomHash(pixel(0), pixel(1))];
  }

  const MapContainer& getContainer() { return data; }

  /**
   * Computes the index of the slot array key.
   */
  size_t topHash(const int x, const int y) {
    return (x / BucketWidth) +
           (y / BucketWidth) * (MaxImageWidth / BucketWidth);
  }

  /**
   * Computes the index of the data inside the bucket.
   */
  size_t bottomHash(const int x, const int y) {
    return (x % BucketWidth) + (y % BucketWidth) * BucketWidth;
  }

 private:
  /// Main container which stores the data.
  MapContainer data;
};  // namespace QDVO

}  // namespace QDVO

