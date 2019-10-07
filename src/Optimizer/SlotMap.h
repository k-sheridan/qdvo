#pragma once

#include <vector>
#include <set>

struct SlotMapKeyBase {
using index_type = size_t;
using generation_type = size_t;

index_type index;
generation_type generation;
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
        size_t dataIndex; // index of the data in the data array.
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
    KeyType insert(DataType value)
    {
        size_t slotIndex;

        if (freeSlots.empty())
        { 
            // add new slot
            slotIndex = slots.size();
            slots.emplace_back();

        } else
        {
            slotIndex = freeSlots.back();
            freeSlots.pop_back();
        }

        // get slot reference
        Slot& slot = slots.at(slotIndex);
        assert(slot.free == true);

        // set up slot
        // set slot to not free
        slot.free = false;
        slot.dataIndex = data.size();

        // push a new data member to the back of the data arrays.
        data.push_back(value);
        dataToSlotIndex.push_back(slotIndex);

        // setup key.
        KeyType key;
        key.index = slotIndex;
        key.generation = slot.generation;

        return key;

    }

    /**
     * O(1)
     */
    void erase(KeyType& key) 
    {
        // check if there are enough slots
        if (slots.size() <= key.index)
        {
            return;
        }

        const auto& slot = slots.at(key.index);

        // check if the generations match
        if (slot.generation != key.generation)
        {
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
    typename std::vector<DataType>::iterator at(KeyType& key)
    {
        // check if there are enough slots
        if (slots.size() <= key.index)
        {
            return data.end();
        }

        const auto& slot = slots.at(key.index);

        // check if the generations match
        if (slot.generation != key.generation)
        {
            return data.end();
        }

        assert(slot.dataIndex < data.size());

        return data.begin() + slot.dataIndex;
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

    /**
     * O(1)
     * number of elements currently stored the slot map.
     */
    size_t size()
    {
        return data.size();
    }

};