#pragma once

#include <cassert>
#include <set>
#include <vector>

namespace ArgMin {

/**
 * Example of a slotmap key which meets the requirements.
 */
struct SlotMapKeyBase {
 public:
  using index_type = size_t;
  using generation_type = size_t;

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

template <typename Index>
struct BaseMetaData {
  /// Vector of occupancy flags.
  std::vector<bool> occupancy;
  /// Vector of slot generations.
  std::vector<Index> generations;

  void clear() {
    occupancy.clear();
    generations.clear();
  }
};

template <typename Index>
struct ContiguousMetaData : BaseMetaData<Index> {
  /// Mappings between the contigous data container and the slots.
  std::vector<Index> slotToDataIndex;
  std::vector<Index> dataToSlotIndex;

  void clear() {
    BaseMetaData<Index>::clear();
    slotToDataIndex.clear();
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

  // Temporarily don't support direct.
  static_assert(!Direct);

  /// Iterator type for this slot map.
  using Iterator = typename DataContainer::iterator;
  using ConstIterator = typename DataContainer::const_iterator;

  using Index = size_t;

  /// Select the metadata type for this slotmap.
  using MetaData =
      select_metadata<Direct, KeyValueInsertion, typename KeyType::generation_type>;

  /// SoA containing the metadata used for bookkeeping.
  MetaData metadata;

  /// Container holding all data.
  DataContainer data;

 public:
  typedef KeyType key_type;
  typedef DataType data_type;

  enum InsertResult { SUCCESS_NO_OVERWRITE, SUCCESS_OVERWRITE, FAILURE };

  SlotMap() {}

  /// Emptys the slot map while retaining its memory.
  void clear() {
    data.clear();
    metadata.clear();
  }

  /**
   * O(1)
   */
  template <typename = typename std::enable_if<!KeyValueInsertion>::type>
  KeyType insert(DataType value) {
    Index slotIndex;

    if (metadata.freeSlots.empty()) {
      // add new slot
      slotIndex = slots.size();
      slots.emplace_back();
    } else {
      slotIndex = freeSlots.back();
      freeSlots.pop_back();
    }

    // get slot reference
    Slot &slot = slots.at(slotIndex);
    assert(slot.free == true);

    // set up slot
    // set slot to not free
    slot.free = false;
    slot.dataIndex = data.size();

    // push a new data member to the back of the data arrays.
    data.push_back(std::move(value));
    dataToSlotIndex.push_back(slotIndex);

    // setup key.
    KeyType key;
    key.index = slotIndex;
    key.generation = slot.generation;

    return key;
  }

  /**
   * O(1)
   * The insert result lets the user know if an element was overwritten, or if a
   * failure occured.
   */
  template <typename = typename std::enable_if<KeyValueInsertion>::type>
  InsertResult insert(KeyType key, DataType value) {
    const size_t slotIndex = key.index;

    // Insert a new slot if necessary.
    if (slotIndex >= slots.size()) {
      slots.resize(slotIndex + 1);
    }

    // get slot reference
    Slot &slot = slots.at(slotIndex);

    // Update the slot generation.
    slot.generation = key.generation;

    // Is the slot free?
    if (slot.free) {
      // set up slot
      // set slot to not free
      slot.free = false;
      slot.dataIndex = data.size();

      // push a new data member to the back of the data arrays.
      data.push_back(std::move(value));
      dataToSlotIndex.push_back(slotIndex);

      return InsertResult::SUCCESS_NO_OVERWRITE;

    } else {
      // Overwrite the slot.
      data.at(slot.dataIndex) = value;
      assert(dataToSlotIndex.at(slot.dataIndex) == slotIndex);

      return InsertResult::SUCCESS_OVERWRITE;
    }
  }

  /**
   * O(1)
   */
  void erase(const KeyType &key) {
    // check if there are enough slots
    if (slots.size() <= key.index) {
      return;
    }

    const auto &slot = slots.at(key.index);

    // check if the generations match
    if (slot.generation != key.generation) {
      return;
    }

    if (slot.free) {
      return;
    }

    assert(slot.dataIndex < data.size());

    // swap the data to be deleted with the last data element.
    std::swap(data.at(slot.dataIndex), data.back());
    std::swap(dataToSlotIndex.at(slot.dataIndex), dataToSlotIndex.back());

    // fix the slot pointing to the swapped data
    slots.at(dataToSlotIndex.at(slot.dataIndex)).dataIndex = slot.dataIndex;

    // increment the deleted slots generation
    const size_t deletedSlotIndex = dataToSlotIndex.back();
    ++slots.at(deletedSlotIndex).generation;

    // pop the data from the back
    data.pop_back();
    dataToSlotIndex.pop_back();

    // free the slot
    slots.at(deletedSlotIndex).free = true;
    freeSlots.push_back(deletedSlotIndex);
  }

  /**
   * O(1)
   * returns end() if key is invalid.
   */
  Iterator at(const KeyType &key) {
    // check if there are enough slots
    if (slots.size() <= key.index) {
      return data.end();
    }

    const auto &slot = slots.at(key.index);

    // check if the generations match
    if (slot.generation != key.generation) {
      return data.end();
    }

    if (slot.free) {
      return data.end();
    }

    assert(slot.dataIndex < data.size());

    return data.begin() + slot.dataIndex;
  }

  ConstIterator at(const KeyType &key) const { return at(key); }

  /**
   * O(1)
   * iterator pointing to the beginning of this containers internal data
   * container.
   */
  Iterator begin() { return data.begin(); }

  ConstIterator begin() const { return data.begin(); }

  /**
   * O(1)
   * iterator pointing to one past the end of this containers internal data
   * container.
   */
  Iterator end() { return data.end(); }

  ConstIterator end() const { return data.end(); }

  /**
   * O(1)
   * number of elements currently stored the slot map.
   */
  size_t size() const { return data.size(); }

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
    if (slots.size() <= key.index) {
      return KeyType();
    }

    auto &slot = slots.at(key.index);

    // check if the generations match
    if (slot.generation != key.generation) {
      return KeyType();
    }

    if (slot.free) {
      return KeyType();
    }

    KeyType newKey = key;

    ++slot.generation;
    newKey.generation = slot.generation;

    return newKey;
  }

  /**
   * O(1)
   * constructs a variable key index and generation from a data index.
   * Asserts that the data index is valid.
   */
  KeyType getKeyFromDataIndex(size_t dataIndex) {
    assert(dataIndex < dataToSlotIndex.size());
    size_t slotIndex = dataToSlotIndex.at(dataIndex);

    KeyType result;

    result.index = slotIndex;
    assert(slotIndex < slots.size());
    result.generation = slots.at(slotIndex).generation;

    return result;
  }

  /**
   * O(1)
   * Given an iterator construct its associated key.
   */
  KeyType getKeyFromIterator(ConstIterator it) {
    return getKeyFromDataIndex(std::distance(begin(), it));
  }
};

}  // namespace ArgMin
