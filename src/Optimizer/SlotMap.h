#pragma once

#include <cassert>
#include <iostream>
#include <set>
#include <vector>

namespace ArgMin {

/**
 * Example of a slotmap key which meets the requirements.
 */
struct SlotMapKeyBase {
 public:
  using index_type = uint32_t;
  using generation_type = uint32_t;

  index_type index = std::numeric_limits<index_type>::max();
  generation_type generation = 0;

  void setInvalid() { index = std::numeric_limits<index_type>::max(); }

  bool isInvalid() { return index == std::numeric_limits<index_type>::max(); }
};

/**
 * Strongly typed slotmap key.
 * This helps prevent bugs where you use the wrong key
 * to access a slotmap.
 */
template <typename T>
struct TypedSlotMapKey : public SlotMapKeyBase {
  /// A compile time helper to get the variable type of this key.
  typedef T variable_type;

  /// Compares two keys by their index and generation.
  bool operator==(const TypedSlotMapKey<T> &other) const {
    return this->index == other.index && this->generation == other.generation;
  }
};

/// Slot information.
template <typename Index>
struct Slot {
  /// Is this slot in use.
  bool occupied;
  /// Generation of this slot.
  Index generation;
};

template <typename Index>
struct BaseMetaData {
  /// Vector of slot information.
  std::vector<Slot<Index>> slots;

  /// Keep track of the number of occupied slots.
  Index size;

  inline void clear() {
    slots.clear();
    size = 0;
  }
};

/// Slot information.
template <typename Index>
struct ContiguousSlot {
  /// Is this slot in use.
  bool occupied;
  /// Generation of this slot.
  Index generation;
  /// Index of data.
  Index dataIndex;
};

template <typename Index>
struct ContiguousMetaData {
  /// Vector of contiguos slot info.
  std::vector<ContiguousSlot<Index>> slots;
  /// Mapping between the contigous data container and the slots.
  std::vector<Index> dataToSlotIndex;

  inline void clear() {
    slots.clear();
    dataToSlotIndex.clear();
  }
};

/// Fast iteration, keys are chosen for you.
template <typename Index>
struct ContiguousValueInsertMetaData : ContiguousMetaData<Index> {
  std::vector<Index> freeSlots;

  void clear() {
    ContiguousMetaData<Index>::clear();
    freeSlots.clear();
  }
};

/// Fast iteration, you choose the key.
template <typename Index>
struct ContiguousKeyValueInsertMetaData : ContiguousMetaData<Index> {
  void clear() { ContiguousMetaData<Index>::clear(); }
};

/// Fast access based on an index, keys are chosen for you.
template <typename Index>
struct DirectValueInsertMetaData : BaseMetaData<Index> {
  std::vector<Index> freeSlots;

  void clear() {
    BaseMetaData<Index>::clear();
    freeSlots.clear();
  }
};

/// Fast access based on an index, you choose the key.
template <typename Index>
struct DirectKeyValueInsertMetaData : BaseMetaData<Index> {
  void clear() { BaseMetaData<Index>::clear(); }
};

/// Helper which selects the metadata used in the slotmap.
template <bool Direct, bool KeyValueInsertion, typename IndexType>
using select_metadata = std::conditional_t<
    Direct,
    std::conditional_t<KeyValueInsertion,
                       DirectKeyValueInsertMetaData<IndexType>,
                       DirectValueInsertMetaData<IndexType>>,
    std::conditional_t<KeyValueInsertion,
                       ContiguousKeyValueInsertMetaData<IndexType>,
                       ContiguousValueInsertMetaData<IndexType>>>;

/**
 * Iterator wrapper
 * This iterator has a ++ operator which searches for an occupied slot.
 */
template <typename SlotVector, typename DataContainer>
class OccupiedSlotIterator {
 public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = typename DataContainer::value_type;
  using difference_type = std::ptrdiff_t;
  using pointer = typename DataContainer::pointer;
  using reference = typename DataContainer::reference;

  OccupiedSlotIterator(typename SlotVector::iterator &&slotIterator,
                       typename SlotVector::iterator &&slotEnd,
                       typename DataContainer::iterator &&dataIterator)
      : slotIterator(slotIterator),
        slotEnd(slotEnd),
        dataIterator(dataIterator) {}

  OccupiedSlotIterator &operator++() {
    // If the slot iterator is at the end don't continue.
    if (slotIterator == slotEnd) {
      return *this;
    }

    // Search for the next occupied slot.
    do {
      ++dataIterator;
      ++slotIterator;
    } while (slotIterator != slotEnd && !slotIterator->occupied);

    return *this;
  }

  OccupiedSlotIterator operator++(int) {
    OccupiedSlotIterator tmp(*this);
    operator++();
    return tmp;
  }

  bool operator==(const OccupiedSlotIterator &rhs) const {
    return slotIterator == rhs.slotIterator;
  }

  bool operator!=(const OccupiedSlotIterator &rhs) const {
    return slotIterator != rhs.slotIterator;
  }

  reference &operator*() {
    assert(slotIterator->occupied);
    return *dataIterator;
  }

  OccupiedSlotIterator operator+(const difference_type &movement) {
    dataIterator += movement;
    slotIterator += movement;
    return *this;
  }

  difference_type operator-(const OccupiedSlotIterator &rawIterator) {
    return std::distance(rawIterator.getDataIterator(),
                         this->getDataIterator());
  }

  typename DataContainer::iterator getDataIterator() const {
    return dataIterator;
  };

 protected:
  typename SlotVector::iterator slotIterator;
  typename SlotVector::iterator slotEnd;
  typename DataContainer::iterator dataIterator;
};

/**
 * ConstIterator wrapper
 * This iterator has a ++ operator which searches for an occupied slot.
 */
template <typename SlotVector, typename DataContainer>
class ConstOccupiedSlotIterator {
 public:
  ConstOccupiedSlotIterator(
      OccupiedSlotIterator<SlotVector, DataContainer> &iterator)
      : it(iterator) {}

  ConstOccupiedSlotIterator &operator++() {
    ++it;
    return *this;
  }

  ConstOccupiedSlotIterator operator++(int) {
    ConstOccupiedSlotIterator tmp(*this);
    operator++();
    return tmp;
  }

  bool operator==(const ConstOccupiedSlotIterator &rhs) const {
    return getDataIterator() == rhs.getDataIterator();
  }

  bool operator!=(const ConstOccupiedSlotIterator &rhs) const {
    return getDataIterator() != rhs.getDataIterator();
  }

  const typename OccupiedSlotIterator<SlotVector, DataContainer>::reference &
  operator*() {
    return *it;
  }

  ConstOccupiedSlotIterator operator+(
      const typename OccupiedSlotIterator<
          SlotVector, DataContainer>::difference_type &movement) {
    return it + movement;
  }

  typename OccupiedSlotIterator<SlotVector, DataContainer>::difference_type
  operator-(const ConstOccupiedSlotIterator &rawIterator) {
    return std::distance(rawIterator.getDataIterator(),
                         this->getDataIterator());
  }

  typename DataContainer::iterator getDataIterator() const {
    return it->getDataIterator;
  };

 protected:
  OccupiedSlotIterator<SlotVector, DataContainer> it;
};

/**
 * SlotMap is an associative container which provides constant time random
 * access, constant time insert, and constant time erase.
 */
template <typename DataType, typename KeyType = TypedSlotMapKey<DataType>,
          typename DataContainer = std::vector<DataType>, bool Direct = false,
          bool KeyValueInsertion = false>
class SlotMap {
  // Ensure that the key meets requirements.
  static_assert(std::is_integral<typename KeyType::index_type>::value);
  static_assert(std::is_integral<typename KeyType::generation_type>::value);

  using Index = size_t;

  /// Select the metadata type for this slotmap.
  using MetaData = select_metadata<Direct, KeyValueInsertion,
                                   typename KeyType::generation_type>;

  /// Slot vector used in this class.
  using SlotVector = decltype(std::declval<MetaData>().slots);

  /// Iterator type for this slot map.
  using Iterator =
      std::conditional_t<Direct,
                         OccupiedSlotIterator<SlotVector, DataContainer>,
                         typename DataContainer::iterator>;
  using ConstIterator =
      std::conditional_t<Direct,
                         ConstOccupiedSlotIterator<SlotVector, DataContainer>,
                         typename DataContainer::const_iterator>;

  /// SoA containing the metadata used for bookkeeping.
  MetaData metadata;

  /// Container holding all data.
  DataContainer data;

 public:
  typedef KeyType key_type;
  typedef DataType data_type;

  using data_container = DataContainer;

  enum InsertResult { SUCCESS_NO_OVERWRITE, SUCCESS_OVERWRITE, FAILURE };

  SlotMap() {
    if constexpr (Direct) {
      metadata.size = 0;
    }
  }

  /// Emptys the slot map while retaining its memory.
  void clear() {
    data.clear();
    metadata.clear();
  }

  /**
   * O(1)
   */
  KeyType insert(DataType value) {
    static_assert(!KeyValueInsertion,
                  "This insertion method is not possible for this type.");

    if constexpr (!Direct) {
      Index slotIndex;

      if (metadata.freeSlots.empty()) {
        // add new slot
        slotIndex = metadata.slots.size();
        metadata.slots.emplace_back();
      } else {
        slotIndex = metadata.freeSlots.back();
        metadata.freeSlots.pop_back();
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[slotIndex];

      // get slot reference
      assert(!slot.occupied == true);

      // set up slot
      // set slot to not free
      slot.occupied = true;
      slot.dataIndex = data.size();

      // push a new data member to the back of the data arrays.
      data.push_back(std::move(value));
      metadata.dataToSlotIndex.push_back(slotIndex);

      // setup key.
      KeyType key;
      key.index = slotIndex;
      key.generation = slot.generation;

      return key;
    } else {
      if (metadata.freeSlots.empty()) {
        // add new slot
        const Index slotIndex = metadata.slots.size();
        metadata.slots.emplace_back();

        // Fetch the slot info.
        auto &slot = metadata.slots[slotIndex];

        // get slot reference
        assert(!slot.occupied == true);
        assert(data.size() == slotIndex);

        // set up slot
        // set slot to not free
        slot.occupied = true;

        // push a new data member to the back of the data arrays.
        data.push_back(value);

        // Increment the size since we added an element.
        ++metadata.size;

        // setup key.
        KeyType key;
        key.index = slotIndex;
        key.generation = slot.generation;

        return key;
      } else {
        const Index slotIndex = metadata.freeSlots.back();
        metadata.freeSlots.pop_back();

        // Fetch the slot info.
        auto &slot = metadata.slots[slotIndex];

        // get slot reference
        assert(!slot.occupied == true);
        assert(data.size() > slotIndex);

        // Increment the size since we added an element.
        ++metadata.size;

        // set up slot
        // set slot to not free
        slot.occupied = true;

        // push a new data member to the back of the data arrays.
        data[slotIndex] = value;

        // setup key.
        KeyType key;
        key.index = slotIndex;
        key.generation = slot.generation;

        return key;
      }
    }
  }

  /**
   * O(1)
   * The insert result lets the user know if an element was overwritten, or if a
   * failure occured.
   */
  InsertResult insert(KeyType key, DataType value) {
    static_assert(KeyValueInsertion,
                  "This insertion method is not possible for this type.");

    if constexpr (!Direct) {
      const Index slotIndex = key.index;

      // Insert a new slot if necessary.
      if (slotIndex >= metadata.slots.size()) {
        metadata.slots.resize(slotIndex + 1);
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[slotIndex];

      // Update the slot generation.
      slot.generation = key.generation;

      // Is the slot free?
      if (!slot.occupied) {
        // set up slot
        // set slot to not free
        slot.occupied = true;
        slot.dataIndex = data.size();

        // push a new data member to the back of the data arrays.
        data.push_back(std::move(value));
        metadata.dataToSlotIndex.push_back(slotIndex);

        return InsertResult::SUCCESS_NO_OVERWRITE;

      } else {
        // Overwrite the slot.
        data[slot.dataIndex] = value;
        assert(metadata.dataToSlotIndex[slot.dataIndex] == slotIndex);

        return InsertResult::SUCCESS_OVERWRITE;
      }
    } else {
      // Insert a new slot if necessary.
      if (key.index >= metadata.slots.size()) {
        metadata.slots.resize(key.index + 1, {false, 0});
        assert(key.index + 1 > data.size());
        data.resize(key.index + 1);
      }

      assert(key.index < data.size());
      assert(key.index < metadata.slots.size());

      // Fetch the slot info.
      auto &slot = metadata.slots[key.index];

      // Update the slot generation.
      slot.generation = key.generation;

      // Is the slot free?
      if (!slot.occupied) {
        // set up slot
        // set slot to not free
        slot.occupied = true;

        // push a new data member to the back of the data arrays.
        data[key.index] = value;

        // Increment the size since we added an element.
        ++metadata.size;

        return InsertResult::SUCCESS_NO_OVERWRITE;

      } else {
        // Overwrite the slot.
        data[key.index] = value;

        return InsertResult::SUCCESS_OVERWRITE;
      }
    }
  }

  /**
   * O(1)
   * This variant of insert requires that the data type has a
   * default constructor.
   *
   * If, data already exists at this key the data is returned by reference.
   * If, the key does not exist, data is emplaced and returned.
   */
  DataType &insert(KeyType key) {
    static_assert(KeyValueInsertion,
                  "This insertion method is not possible for this type.");
    if constexpr (!Direct) {
      const Index slotIndex = key.index;

      // Insert a new slot if necessary.
      if (slotIndex >= metadata.slots.size()) {
        metadata.slots.resize(slotIndex + 1);
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[slotIndex];

      // Update the slot generation.
      slot.generation = key.generation;

      // Is the slot free?
      if (!slot.occupied) {
        // set up slot
        // set slot to not free
        slot.occupied = true;
        slot.dataIndex = data.size();

        // push a new data member to the back of the data arrays.
        data.emplace_back();
        metadata.dataToSlotIndex.push_back(slotIndex);
      }

      return data[slot.dataIndex];
    } else {
      // Insert a new slot if necessary.
      if (key.index >= metadata.slots.size()) {
        metadata.slots.resize(key.index + 1, {false, 0});
        assert(data.size() != key.index + 1);
        data.resize(key.index + 1);
      }

      assert(key.index < data.size());

      // Fetch the slot info.
      auto &slot = metadata.slots[key.index];

      // Update the slot generation.
      slot.generation = key.generation;

      // Is the slot free?
      if (!slot.occupied) {
        // set up slot
        // set slot to not free
        slot.occupied = true;

        // Increment the size since we added an element.
        ++metadata.size;
      }

      return data[key.index];
    }
  }

  /**
   * O(1)
   */
  void erase(const KeyType &key) {
    if constexpr (!Direct) {
      // check if there are enough slots
      if (metadata.slots.size() <= key.index) {
        return;
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[key.index];

      // check if the generations match
      if (slot.generation != key.generation) {
        return;
      }

      if (!slot.occupied) {
        return;
      }

      assert(slot.dataIndex < data.size());

      // swap the data to be deleted with the last data element.
      std::swap(data[slot.dataIndex], data.back());
      std::swap(metadata.dataToSlotIndex[slot.dataIndex],
                metadata.dataToSlotIndex.back());

      // fix the slot pointing to the swapped data
      metadata.slots[metadata.dataToSlotIndex[slot.dataIndex]].dataIndex =
          slot.dataIndex;

      // increment the deleted slots generation
      const Index deletedSlotIndex = metadata.dataToSlotIndex.back();
      auto &deletedSlot = metadata.slots[deletedSlotIndex];
      ++(deletedSlot.generation);

      // pop the data from the back
      data.pop_back();
      metadata.dataToSlotIndex.pop_back();

      // free the slot
      deletedSlot.occupied = false;
      // If this is not a slot array, push back to the free slot list.
      if constexpr (!KeyValueInsertion) {
        metadata.freeSlots.push_back(deletedSlotIndex);
      }
    } else {
      // check if there are enough slots
      if (metadata.slots.size() <= key.index) {
        return;
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[key.index];

      // check if the generations match
      if (slot.generation != key.generation) {
        return;
      }

      if (!slot.occupied) {
        return;
      }

      // Free the slot.
      slot.occupied = false;

      // Increment the generation.
      ++(slot.generation);

      // Decrement the size since we are removing an element.
      --(metadata.size);

      if constexpr (!KeyValueInsertion) {
        metadata.freeSlots.push_back(key.index);
        assert((metadata.slots.size() - metadata.freeSlots.size()) ==
               metadata.size);
      }
    }
  }

  /**
   * O(1)
   * returns end() if key is invalid.
   */
  Iterator at(const KeyType &key) {
    if constexpr (!Direct) {
      // check if there are enough slots
      if (metadata.slots.size() <= key.index) {
        return data.end();
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[key.index];

      // check if the generations match
      if (slot.generation != key.generation) {
        return data.end();
      }

      if (!slot.occupied) {
        return data.end();
      }

      assert(slot.dataIndex < data.size());

      return data.begin() + slot.dataIndex;
    } else {
      // check if there are enough slots
      if (metadata.slots.size() <= key.index) {
        return end();
      }

      // Fetch the slot info.
      auto &slot = metadata.slots[key.index];

      // check if the generations match
      if (slot.generation != key.generation) {
        return end();
      }

      if (!slot.occupied) {
        return end();
      }

      assert(key.index < data.size());

      return {metadata.slots.begin() + key.index, metadata.slots.end(),
              data.begin() + key.index};
    }
  }

  ConstIterator at(const KeyType &key) const { return at(key); }

  /**
   * O(1)
   * iterator pointing to the beginning of this containers internal data
   * container.
   */
  Iterator begin() {
    if constexpr (!Direct) {
      return data.begin();
    } else {
      Iterator it = {metadata.slots.begin(), metadata.slots.end(),
                     data.begin()};

      // If the first slot is not occupied iterate.
      if (!metadata.slots.front().occupied) {
        ++it;
      }

      return it;
    }
  }

  ConstIterator begin() const { return begin(); }

  /**
   * O(1)
   * iterator pointing to one past the end of this containers internal data
   * container.
   */
  Iterator end() {
    if constexpr (!Direct) {
      return data.end();
    } else {
      return {metadata.slots.end(), metadata.slots.end(), data.end()};
    }
  }

  ConstIterator end() const { return end(); }

  /**
   * O(1)
   * number of elements currently stored the slot map.
   */
  size_t size() const {
    if constexpr (!Direct) {
      return data.size();
    } else {
      return metadata.size;
    }
  }

  /**
   * O(1)
   *
   * Increments the generation of a slot to invalidate all previously returned
   * keys. The key must be a valid key, otherwise an invalid key is returned.
   *
   * Equivalent to:
   * data = at(key1);
   * erase(key1);
   * key2 = insert(data);
   */
  KeyType updateSlotGeneration(const KeyType &key) {
    // check if there are enough slots
    if (metadata.slots.size() <= key.index) {
      return KeyType();
    }

    // Fetch the slot info.
    auto &slot = metadata.slots[key.index];

    // check if the generations match
    if (slot.generation != key.generation) {
      return KeyType();
    }

    if (!slot.occupied) {
      return KeyType();
    }

    KeyType newKey = key;

    newKey.generation = ++slot.generation;

    return newKey;
  }

  /**
   * O(1)
   * Given an iterator construct its associated key.
   */
  KeyType getKeyFromIterator(Iterator it) {
    if constexpr (!Direct) {
      return getKeyFromDataIndex(std::distance(begin(), it));
    } else {
      return getKeyFromDataIndex(
          std::distance(data.begin(), it.getDataIterator()));
    }
  }

 private:
  /**
   * O(1)
   * constructs a variable key index and generation from a data index.
   * Asserts that the data index is valid.
   */
  KeyType getKeyFromDataIndex(size_t dataIndex) {
    if constexpr (!Direct) {
      assert(dataIndex < metadata.dataToSlotIndex.size());
      size_t slotIndex = metadata.dataToSlotIndex[dataIndex];

      KeyType result;

      result.index = slotIndex;
      assert(slotIndex < metadata.slots.size());
      result.generation = metadata.slots[slotIndex].generation;

      return result;
    } else {
      assert(dataIndex < metadata.slots.size());

      KeyType result;

      result.index = dataIndex;
      result.generation = metadata.slots[dataIndex].generation;

      return result;
    }
  }
};

}  // namespace ArgMin
