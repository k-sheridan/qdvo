#pragma once

#include <vector>
#include <set>

template <typename Index, typename Generation>
struct SlotMapKeyBase {
using index_type = Index;
using generation_type = Generation;

Index index;
Generation generation;
};

/**
 * This is a slot map as defined by allan deutsch.
 */
template <typename DataType, typename KeyType>
class SlotMap
{

    static_assert(std::is_integral<typename KeyType::index_type>::value);
    static_assert(std::is_integral<typename KeyType::generation_type>::value);

    struct Slot
    {
        typename KeyType::index_type index = 0;
        typename KeyType::generation_type generation = 0;
        bool free = true;
    };

    std::vector<Slot> slots; // Set of keys currently in use.
    std::vector<size_t> freeSlots; // Stores indices of which slots can be inserted to.

    std::vector<DataType> data;  // vector containing all data.
    std::vector<size_t> dataToSlotIndex; // vector which points data to its slot index.
    

public:
    SlotMap()
    {
    }

    /**
     * O(1)
     */
    KeyType insert(DataType&& data)
    {

    }

    /**
     * O(1)
     */
    void erase(KeyType& key) 
    {

    }

    /**
     * O(1)
     * returns end() if key is invalid.
     */
    typename std::vector<DataType>::iterator at(KeyType& key)
    {
        
    }

    /**
     * O(1)
     * iterator pointing to the beginning of this containers internal data container.
     */
    typename std::vector<DataType>::iterator begin()
    {
        return data.begin();
    }

    /**
     * O(1)
     * iterator pointing to one past the end of this containers internal data container.
     */
    typename std::vector<DataType>::iterator end()
    {
        return data.end();
    }

};